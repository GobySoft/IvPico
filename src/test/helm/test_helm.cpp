// Copyright 2026:
//   GobySoft, LLC
//
// This file is part of IvPico.
//
// IvPico is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 2 of the License, or (at your option) any later
// version.
//
// IvPico is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with IvPico. If not, see <https://www.gnu.org/licenses/>.

// Scenario tests for ivpico::helm::Helm, the MOOS-free port of pHelmIvP.
//
// Each scenario writes a small .bhv file, configures a helm with a course and
// speed domain, and drives it with a fake clock: inputs go in with
// Helm::handle(), then Helm::iterate(t) runs one helm iteration at time t, as
// pHelmIvP handles its mail and then iterates. Where a scenario needs the
// vehicle to move, a simple kinematic model turns the helm's decisions into the
// next Navigation input.
//
// Prints one line per failed check and exits non-zero if there were any.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <string>

#include "helm/conversions.h"
#include "helm/helm.h"

using ivp::interface::Command;
using ivp::interface::HelmConfig;
using ivp::interface::Navigation;
using ivp::interface::Posting;
using ivp::interface::Solution;
using ivp::interface::Status;
using ivp::interface::Variable;
using ivpico::helm::Helm;
using ivpico::helm::Output;

namespace
{
int errors = 0;

void check(bool ok, const std::string& scenario, const std::string& what)
{
    if (!ok)
    {
        std::cerr << "failed: " << scenario << ": " << what << "\n";
        ++errors;
    }
}

constexpr double t0 = 1.7e9;
constexpr double dt = 0.25;

std::string write_bhv(const std::string& name, const std::string& contents)
{
    const auto path = std::filesystem::temp_directory_path() / ("ivpico_test_" + name + ".bhv");
    std::ofstream(path) << contents;
    return path.string();
}

HelmConfig config(const std::string& bhv_file)
{
    HelmConfig cfg;
    cfg.add_behavior_file(bhv_file);
    cfg.set_ownship("alpha");
    auto* course = cfg.add_domain();
    course->set_name("course");
    course->set_low(0);
    course->set_high(359);
    course->set_points(360);
    auto* speed = cfg.add_domain();
    speed->set_name("speed");
    speed->set_low(0);
    speed->set_high(4);
    speed->set_points(41);
    return cfg;
}

struct Vehicle
{
    double x{0}, y{0}, heading{0}, speed{0};

    Navigation navigation(double t) const
    {
        Navigation nav;
        nav.set_self(true);
        auto& n = *nav.mutable_nav();
        n.set_time(static_cast<int64_t>(t * 1e6));
        n.mutable_enu()->set_e(x);
        n.mutable_enu()->set_n(y);
        n.mutable_attitude()->set_heading(heading * M_PI / 180);
        n.add_speed()->set_value(speed);
        return nav;
    }

    // Turn and change speed at once, then move for dt
    void follow(const Solution& solution)
    {
        const auto& desired = solution.desired();
        if (desired.has_attitude() && desired.attitude().has_heading())
            heading = desired.attitude().heading() * 180 / M_PI;
        if (desired.has_speed() && desired.speed().has_value())
            speed = desired.speed().value();
        x += speed * dt * std::sin(heading * M_PI / 180);
        y += speed * dt * std::cos(heading * M_PI / 180);
    }
};

Variable variable(const std::string& key, const std::string& value)
{
    Variable v;
    v.set_key(key);
    v.set_sval(value);
    return v;
}

Command manual_override(bool on)
{
    Command c;
    c.set_manual_override(on);
    return c;
}

double desired_course(const Solution& s) { return s.desired().attitude().heading() * 180 / M_PI; }

bool has_posting(const Output& out, const std::string& key, const std::string& value,
                 Posting::Origin origin)
{
    for (const auto& p : out.postings)
        if (p.key() == key && p.sval() == value && p.origin() == origin)
            return true;
    return false;
}

std::string mode(const Output& out)
{
    for (const auto& m : out.status.mode())
        if (m.variable() == "MODE")
            return m.value();
    return "";
}

// The s1_alpha pattern: initialize DEPLOY and RETURN, two modes, a survey
// waypoint whose endflag sets RETURN, and a return waypoint. Checks:
//  - "initialize" postings on the first iteration;
//  - no decision before navigation arrives, and PARK until the override is lifted;
//  - after DEPLOY and the override, DRIVE in mode ACTIVE:SURVEYING, heading north;
//  - reaching the waypoint posts RETURN=true.
// On the iteration the survey completes, no behavior produces an objective
// function, so the helm declares NothingToDo. That's an allstop pHelmIvP
// rides out for an iteration, and park_on_allstop only parks on errors, so
// with it set or not, RETURN=true then switches to ACTIVE:RETURNING and turns
// the vehicle around without parking.
void waypoint_mission(bool park_on_allstop)
{
    const std::string scenario =
        std::string("waypoint_mission (park_on_allstop: ") + (park_on_allstop ? "true)" : "false)");
    auto cfg = config(write_bhv("waypoint_mission", R"(
initialize DEPLOY = false
initialize RETURN = false

set MODE = ACTIVE {
  DEPLOY = true
} INACTIVE

set MODE = SURVEYING {
  MODE = ACTIVE
  RETURN != true
} RETURNING

Behavior = BHV_Waypoint
{
  name      = waypt_survey
  pwt       = 100
  condition = MODE == SURVEYING
  endflag   = RETURN = true
  perpetual = true
  speed     = 2
  capture_radius = 5
  points    = 0,40
}

Behavior = BHV_Waypoint
{
  name      = waypt_return
  pwt       = 100
  condition = MODE == RETURNING
  speed     = 2
  capture_radius = 5
  points    = 0,-40
}
)"));
    cfg.set_park_on_allstop(park_on_allstop);
    Helm helm(cfg);
    check(helm.config_warnings().empty(), scenario, "config warnings");

    double t = t0;
    const Output& first = helm.iterate(t);
    check(has_posting(first, "DEPLOY", "false", Posting::INITIALIZE), scenario,
          "no initialize DEPLOY=false");
    check(!first.solution, scenario, "solution before navigation");
    check(first.status.state() == Status::PARK, scenario, "not PARK at start");

    Vehicle vehicle;
    helm.handle(vehicle.navigation(t));
    helm.handle(variable("DEPLOY", "true"));
    helm.handle(manual_override(false));

    bool surveying = false, returned = false, turned = false, parked = false;
    for (int i = 0; i < 400; ++i)
    {
        t += dt;
        const Output& out = helm.iterate(t);
        if (out.solution && out.solution->allstop() == ivp::interface::CLEAR)
        {
            if (mode(out) == "ACTIVE:SURVEYING" && std::fabs(desired_course(*out.solution)) < 2)
                surveying = true;
            if (mode(out) == "ACTIVE:RETURNING" &&
                std::fabs(desired_course(*out.solution) - 180) < 2)
                turned = true;
            vehicle.follow(*out.solution);
        }
        for (const auto& p : out.postings)
            if (p.key() == "RETURN" && p.sval() == "true" && p.behavior() == "waypt_survey")
                returned = true;
        if (returned && out.status.state() == Status::PARK)
            parked = true;
        helm.handle(vehicle.navigation(t));
    }
    check(surveying, scenario, "never surveyed north in DRIVE");
    check(returned, scenario, "waypt_survey never posted its endflag RETURN=true");
    check(turned, scenario, "never turned south in mode ACTIVE:RETURNING");
    check(!parked, scenario, "parked after the survey");
}

// One behavior posts an update for another: a timer's endflag posts
// CH_UPDATES = heading=180 to a constant-heading behavior that started at 90.
// pHelmIvP only sees that update when the MOOSDB echoes the timer's post
// back, on the next iteration, and the helm library emulates that echo.
// Checks that the course is 90 until the update is posted, still 90 on the
// iteration that posts it, and 180 on the next.
void update_through_echo()
{
    const std::string scenario = "update_through_echo";
    auto cfg = config(write_bhv(scenario, R"(
Behavior = BHV_ConstantHeading
{
  name     = ch
  heading  = 90
  duration = no-time-limit
  updates  = CH_UPDATES
}

Behavior = BHV_Timer
{
  name     = timer
  duration = 3
  endflag  = CH_UPDATES = heading=180
}
)"));
    cfg.mutable_domain(1)->set_optional(true);
    cfg.set_start_in_drive(true);
    Helm helm(cfg);
    check(helm.config_warnings().empty(), scenario, "config warnings");

    Vehicle vehicle;
    double t = t0;
    int posted_at = -1, turned_at = -1;
    for (int i = 0; i < 40; ++i)
    {
        helm.handle(vehicle.navigation(t));
        const Output& out = helm.iterate(t);
        for (const auto& p : out.postings)
            if (p.key() == "CH_UPDATES" && posted_at < 0)
                posted_at = i;
        if (out.solution && out.solution->allstop() == ivp::interface::CLEAR && turned_at < 0 &&
            std::fabs(desired_course(*out.solution) - 180) < 1)
            turned_at = i;
        if (out.solution && posted_at >= 0 && i == posted_at)
            check(std::fabs(desired_course(*out.solution) - 90) < 1, scenario,
                  "course changed on the iteration that posted the update");
        t += dt;
    }
    check(posted_at >= 0, scenario, "timer never posted CH_UPDATES");
    check(turned_at == posted_at + 1, scenario,
          "course 180 on iteration " + std::to_string(turned_at) + ", not one after the post (" +
              std::to_string(posted_at) + ")");
}

// A template behavior spawns when its updates variable names a new instance.
// Checks for a SPAWN life event and the spawned behavior's decision.
void spawn()
{
    const std::string scenario = "spawn";
    auto cfg = config(write_bhv(scenario, R"(
Behavior = BHV_ConstantHeading
{
  name       = ch_
  templating = spawn
  updates    = SPAWN_CH
  duration   = no-time-limit
  heading    = 0
}
)"));
    cfg.mutable_domain(1)->set_optional(true);
    cfg.set_start_in_drive(true);
    cfg.set_park_on_allstop(false);
    Helm helm(cfg);
    check(helm.config_warnings().empty(), scenario, "config warnings");

    Vehicle vehicle;
    double t = t0;
    bool spawned = false, decided = false;
    for (int i = 0; i < 20; ++i)
    {
        helm.handle(vehicle.navigation(t));
        if (i == 8)
            helm.handle(variable("SPAWN_CH", "name=one # heading=45"));
        const Output& out = helm.iterate(t);
        for (const auto& e : out.life_events)
            if (e.type() == ivp::interface::LifeEvent::SPAWN && e.behavior() == "ch_one")
                spawned = true;
        if (out.solution && out.solution->allstop() == ivp::interface::CLEAR &&
            std::fabs(desired_course(*out.solution) - 45) < 1)
            decided = true;
        t += dt;
    }
    check(spawned, scenario, "no SPAWN life event for ch_one");
    check(decided, scenario, "the spawned behavior never decided course 45");
}

Navigation contact(const std::string& name, double x, double y, double heading, double speed,
                   double t)
{
    Navigation nav;
    nav.set_self(false);
    nav.set_type("kayak");
    auto& n = *nav.mutable_nav();
    n.mutable_vehicle()->set_name(name);
    n.set_time(static_cast<int64_t>(t * 1e6));
    n.mutable_enu()->set_e(x);
    n.mutable_enu()->set_n(y);
    n.mutable_attitude()->set_heading(heading * M_PI / 180);
    n.add_speed()->set_value(speed);
    return nav;
}

// The built-in contact manager (pContactMgrV20's alerts) spawns a collision
// avoidance behavior. The avd_ template posts a BCM_ALERT_REQUEST on helm
// start (alert_range 40, cpa_range 60). Ownship holds course 90 at 1 m/s
// from the origin; "bob" starts 150 m east heading west at 2 m/s, and "ann"
// stays 500 m north. Checks:
//  - the CONTACT_INFO alert for bob, as a CONTACT_MANAGER posting, once bob
//    is inside the cpa_range, and none for ann;
//  - a SPAWN life event for avd_bob after that;
//  - a post to the configured disable_var gives BHV_ABLE_FILTER;
//  - with contact_alerts false, neither the alert nor the spawn.
void contact_alert(bool contact_alerts)
{
    const std::string scenario =
        std::string("contact_alert") + (contact_alerts ? "" : " (contact_alerts false)");
    auto cfg = config(write_bhv("contact_alert", R"(
Behavior = BHV_ConstantHeading
{
  name     = ch
  pwt      = 100
  heading  = 90
  duration = no-time-limit
}
Behavior = BHV_ConstantSpeed
{
  name     = cs
  pwt      = 100
  speed    = 1
  duration = no-time-limit
}
Behavior = BHV_AvoidCollision
{
  name       = avd_
  pwt        = 200
  templating = spawn
  updates    = CONTACT_INFO
  contact    = to-be-set
  on_no_contact_ok = true
  pwt_outer_dist = 40
  pwt_inner_dist = 20
  completed_dist = 60
  min_util_cpa_dist = 8
  max_util_cpa_dist = 25
}
)"));
    cfg.set_start_in_drive(true);
    cfg.set_contact_alerts(contact_alerts);
    cfg.mutable_contact_manager()->set_disable_var("XYZ_DISABLE_TARGET");
    Helm helm(cfg);
    check(helm.config_warnings().empty(), scenario, "config warnings");

    Vehicle vehicle;
    double t = t0;
    double bob_x = 150;
    int alert_tick = -1, spawn_tick = -1;
    bool ann_alerted = false, able_filter = false;
    for (int i = 0; i < 200; ++i)
    {
        helm.handle(vehicle.navigation(t));
        helm.handle(contact("bob", bob_x, 0, 270, 2, t));
        helm.handle(contact("ann", 0, 500, 90, 0, t));
        if (i == 150)
            helm.handle(variable("XYZ_DISABLE_TARGET", "bob"));
        const Output& out = helm.iterate(t);
        for (const auto& p : out.postings)
        {
            if (p.origin() != Posting::CONTACT_MANAGER)
                continue;
            if (p.key() == "CONTACT_INFO" && p.sval() == "name=bob # contact=bob" && alert_tick < 0)
                alert_tick = i;
            if (p.key() == "CONTACT_INFO" && p.sval().find("ann") != std::string::npos)
                ann_alerted = true;
            if (p.key() == "BHV_ABLE_FILTER" && p.sval() == "contact=bob,action=disable")
                able_filter = true;
        }
        for (const auto& e : out.life_events)
            if (e.type() == ivp::interface::LifeEvent::SPAWN && e.behavior() == "avd_bob")
                spawn_tick = i;
        if (out.solution)
            vehicle.follow(*out.solution);
        bob_x -= 2 * dt;
        t += dt;
    }

    check(!ann_alerted, scenario, "alerted on ann, 500 m away");
    if (!contact_alerts)
    {
        check(alert_tick < 0, scenario, "the helm raised an alert itself");
        check(spawn_tick < 0, scenario, "spawned avd_bob without an alert");
        return;
    }
    // bob closes at 3 m/s from 150 m, so he's inside 60 m after 30 s (120 iterations)
    check(alert_tick > 100 && alert_tick < 130, scenario,
          "CONTACT_INFO for bob at iteration " + std::to_string(alert_tick));
    check(spawn_tick > alert_tick, scenario, "no SPAWN for avd_bob after the alert");
    check(able_filter, scenario, "no BHV_ABLE_FILTER from XYZ_DISABLE_TARGET");
}

// Behaviors' VIEW_* and NODE_MESSAGE_LOCAL posts come out typed. A waypoint
// behavior draws its track point (VIEW_POINT) and its path (VIEW_SEGLIST), and
// a timer's endflag posts a NODE_MESSAGE_LOCAL. Checks:
//  - a Visualization with a point and one with a seglist of the 3 waypoints,
//    each from waypt;
//  - an OffboardMessage to all, with key FOO and sval "bar";
//  - no VIEW_POINT, VIEW_SEGLIST or NODE_MESSAGE_LOCAL Posting.
void typed_outputs()
{
    const std::string scenario = "typed_outputs";
    auto cfg = config(write_bhv(scenario, R"(
Behavior = BHV_Waypoint
{
  name   = waypt
  speed  = 2
  points = 0,40:40,40:40,0
}
Behavior = BHV_Timer
{
  name     = timer
  duration = 2
  endflag  = NODE_MESSAGE_LOCAL = src_node=alpha,dest_node=all,var_name=FOO,string_val=bar
}
)"));
    cfg.set_start_in_drive(true);
    Helm helm(cfg);
    check(helm.config_warnings().empty(), scenario, "config warnings");

    Vehicle vehicle;
    double t = t0;
    bool point = false, seglist = false, offboard = false, view_posting = false;
    for (int i = 0; i < 40; ++i)
    {
        helm.handle(vehicle.navigation(t));
        const Output& out = helm.iterate(t);
        for (const auto& v : out.visualizations)
        {
            if (v.behavior() != "waypt")
                continue;
            point = point || v.has_point();
            seglist = seglist || (v.has_seglist() && v.seglist().vertex_size() == 3);
        }
        for (const auto& m : out.offboard_messages)
            offboard = offboard ||
                       (m.destination_node() == "all" && m.key() == "FOO" && m.sval() == "bar");
        for (const auto& p : out.postings)
            view_posting = view_posting || p.key() == "VIEW_POINT" || p.key() == "VIEW_SEGLIST" ||
                           p.key() == "NODE_MESSAGE_LOCAL";
        if (out.solution)
            vehicle.follow(*out.solution);
        t += dt;
    }
    check(point, scenario, "no point Visualization from waypt");
    check(seglist, scenario, "no seglist Visualization of the 3 waypoints");
    check(offboard, scenario, "no OffboardMessage FOO=bar to all");
    check(!view_posting, scenario, "a VIEW_* or NODE_MESSAGE_LOCAL Posting");
}

// The Configuration output: on the first iteration, with the course and speed
// domains, and the subscriptions (e.g. MOOS_MANUAL_OVERRIDE); then only when
// something changes. The spawn template's updates variable (SPAWN_CH) is a
// subscription too.
void configuration()
{
    const std::string scenario = "configuration";
    auto cfg = config(write_bhv(scenario, R"(
Behavior = BHV_ConstantHeading
{
  name       = ch_
  templating = spawn
  updates    = SPAWN_CH
  duration   = no-time-limit
  heading    = 0
}
)"));
    Helm helm(cfg);
    Vehicle vehicle;
    double t = t0;
    int count = 0;
    std::optional<ivp::interface::Configuration> first;
    for (int i = 0; i < 10; ++i)
    {
        helm.handle(vehicle.navigation(t));
        const Output& out = helm.iterate(t);
        if (out.configuration)
        {
            ++count;
            if (!first)
                first = *out.configuration;
        }
        t += dt;
    }
    check(first.has_value(), scenario, "no Configuration");
    if (!first)
        return;
    check(count == 1, scenario, std::to_string(count) + " Configurations, unchanged");
    check(first->domain_size() == 2 && first->domain(0).name() == "course" &&
              first->domain(0).points() == 360,
          scenario, "domains: " + first->ShortDebugString());
    auto subscribed = [&](const std::string& var)
    {
        return std::find(first->subscription().begin(), first->subscription().end(), var) !=
               first->subscription().end();
    };
    check(subscribed("MOOS_MANUAL_OVERRIDE"), scenario, "not subscribed to MOOS_MANUAL_OVERRIDE");
    check(subscribed("SPAWN_CH"), scenario, "not subscribed to SPAWN_CH");
}

// RESTART_HELM rebuilds the behaviors from the same files, read again.
// The behavior file asks for course 90; it's rewritten to ask for 180, and:
//  - with RESTART_HELM = type1, the course becomes 180, and the "initialize"
//    variable isn't posted again;
//  - with Command.restart REBUILD_AND_REINITIALIZE (type2), it is.
void restart(bool reinitialize)
{
    const std::string scenario =
        std::string("restart (") + (reinitialize ? "type2" : "type1") + ")";
    const std::string name = std::string("restart_") + (reinitialize ? "2" : "1");
    auto bhv = [](int heading)
    {
        return "initialize FOO = 1\nBehavior = BHV_ConstantHeading\n{\n  name = ch\n  heading = " +
               std::to_string(heading) + "\n  duration = no-time-limit\n}\n";
    };
    auto cfg = config(write_bhv(name, bhv(90)));
    cfg.mutable_domain(1)->set_optional(true);
    cfg.set_start_in_drive(true);
    Helm helm(cfg);

    Vehicle vehicle;
    double t = t0;
    bool course_90 = false, course_180 = false;
    int initializes = 0;
    for (int i = 0; i < 30; ++i)
    {
        helm.handle(vehicle.navigation(t));
        if (i == 10)
        {
            write_bhv(name, bhv(180));
            if (reinitialize)
            {
                Command c;
                c.mutable_restart()->set_type(Command::Restart::REBUILD_AND_REINITIALIZE);
                helm.handle(c);
            }
            else
                helm.handle(variable("RESTART_HELM", "type1"));
        }
        const Output& out = helm.iterate(t);
        for (const auto& p : out.postings)
            if (p.origin() == Posting::INITIALIZE && p.key() == "FOO")
                ++initializes;
        if (out.solution && out.solution->allstop() == ivp::interface::CLEAR)
        {
            const double course = desired_course(*out.solution);
            if (i < 10 && std::fabs(course - 90) < 1)
                course_90 = true;
            if (i > 10 && std::fabs(course - 180) < 1)
                course_180 = true;
        }
        t += dt;
    }
    check(course_90, scenario, "never course 90 before the restart");
    check(course_180, scenario, "never course 180 after the restart");
    check(initializes == (reinitialize ? 2 : 1), scenario,
          "FOO initialized " + std::to_string(initializes) + " times");
}

// Inputs more than ok_skew (here 10 s) from the helm's clock are ignored, with
// a warning; others go in. DEPLOY is set with a timestamp 30 s old, then with
// a current one: only the second turns MODE to ACTIVE.
void ok_skew()
{
    const std::string scenario = "ok_skew";
    auto cfg = config(write_bhv(scenario, R"(
initialize DEPLOY = false
set MODE = ACTIVE {
  DEPLOY = true
} INACTIVE
Behavior = BHV_ConstantHeading
{
  name     = ch
  heading  = 90
  duration = no-time-limit
}
)"));
    cfg.mutable_domain(1)->set_optional(true);
    cfg.set_start_in_drive(true);
    cfg.set_ok_skew(10);
    Helm helm(cfg);

    Vehicle vehicle;
    double t = t0;
    bool warned = false, active_early = false, active = false;
    for (int i = 0; i < 30; ++i)
    {
        helm.handle(vehicle.navigation(t));
        Variable deploy = variable("DEPLOY", "true");
        if (i == 10)
        {
            deploy.set_time(static_cast<int64_t>((t - 30) * 1e6));
            helm.handle(deploy);
        }
        if (i == 20)
        {
            deploy.set_time(static_cast<int64_t>(t * 1e6));
            helm.handle(deploy);
        }
        const Output& out = helm.iterate(t);
        for (const auto& e : out.behavior_events)
            warned = warned || e.text().find("Helm ignores DEPLOY due to skew") == 0;
        if (mode(out) == "ACTIVE")
        {
            if (i < 20)
                active_early = true;
            active = true;
        }
        t += dt;
    }
    check(warned, scenario, "no skew warning for the old DEPLOY");
    check(!active_early, scenario, "acted on the old DEPLOY");
    check(active, scenario, "never ACTIVE after the current DEPLOY");
}

// The VIEW_* and NODE_MESSAGE_LOCAL conversions go both ways: each spec, as
// pHelmIvP's behaviors post them, parses to a Visualization (or
// OffboardMessage) that converts back to a spec parsing to the same message.
void conversion_round_trip()
{
    const std::string scenario = "conversion_round_trip";
    using ivpico::helm::visualization;
    for (const auto& [var, spec] : std::vector<std::pair<std::string, std::string>>{
             {"VIEW_POINT", "x=22.77,y=-27.59,label=alpha's trackpt,label_color=yellow,"
                            "vertex_color=yellow,vertex_size=4"},
             {"VIEW_SEGLIST", "pts={23.8,-80.1:103.2,-70.1},label=abe_avdcol_ben,"
                              "label_color=white,edge_color=white,duration=100"},
             {"VIEW_SEGLIST", "pts={0,0:9,0:0,9},label=abe_avdcol_ben,active=false"},
             {"VIEW_POLYGON", "pts={0,-55:14.1,-60.9:20,-75:14.1,-89.1:0,-95},label=abe_LOITER,"
                              "edge_color=orange,vertex_color=white,edge_size=1"},
             {"VIEW_SEGLR", "pts={151.6,-113.7:151.6,-113.6},ray=11,label=ben_seglr,"
                            "vertex_color=lime_green,vertex_size=2,ray_len=20,head_size=1"}})
    {
        const auto first = visualization(var, spec);
        check(first.has_value(), scenario, "doesn't parse: " + spec);
        if (!first)
            continue;
        const auto [back_var, back_spec] = ivpico::helm::view_posting(*first);
        const auto second = visualization(back_var, back_spec);
        check(back_var == var && second && second->ShortDebugString() == first->ShortDebugString(),
              scenario, spec + " came back as " + back_var + " " + back_spec);
    }

    const std::string msg_spec =
        "src_node=henry,dest_node=gilda,var_name=MOVE_ON,string_val=true,src_app=pHelmIvP,"
        "src_bhv=ping";
    const auto msg = ivpico::helm::offboard_message(msg_spec);
    check(msg && msg->key() == "MOVE_ON" && msg->sval() == "true" &&
              msg->destination_node() == "gilda" && msg->source_behavior() == "ping",
          scenario, "NODE_MESSAGE_LOCAL: " + (msg ? msg->ShortDebugString() : "no parse"));
    if (msg)
    {
        const auto again = ivpico::helm::offboard_message(ivpico::helm::node_message_spec(*msg));
        check(again && again->ShortDebugString() == msg->ShortDebugString(), scenario,
              "NODE_MESSAGE_LOCAL came back as " + ivpico::helm::node_message_spec(*msg));
    }
}

// Allstop reasons and park-on-allstop:
//  - no speed decision with a required speed domain gives
//    MISSING_DECISION_VARIABLES. pHelmIvP gives a missing decision an
//    iteration's grace, and park_on_allstop parks only on errors, so the helm
//    stays in DRIVE and posts no solution while the decision is missing;
//  - with speed optional, the same behaviors give a CLEAR solution;
//  - manual override in DRIVE gives MANUAL_OVERRIDE and PARK.
void allstop()
{
    const std::string scenario = "allstop";
    const std::string bhv = write_bhv(scenario, R"(
Behavior = BHV_ConstantHeading
{
  name     = ch
  heading  = 90
  duration = no-time-limit
}
)");
    {
        auto cfg = config(bhv);
        cfg.set_start_in_drive(true);
        Helm helm(cfg);
        Vehicle vehicle;
        double t = t0;
        bool missing = false, solution = false;
        for (int i = 0; i < 12; ++i)
        {
            helm.handle(vehicle.navigation(t));
            const Output& out = helm.iterate(t);
            if (out.status.allstop() == ivp::interface::MISSING_DECISION_VARIABLES &&
                out.status.missing_decision_variables_size() == 1 &&
                out.status.missing_decision_variables(0) == "speed")
                missing = true;
            if (out.solution)
                solution = true;
            t += dt;
        }
        check(missing, scenario, "no MISSING_DECISION_VARIABLES for speed");
        check(!solution, scenario, "a solution without a speed decision");
        check(helm.state() == Status::DRIVE, scenario, "parked on a missing decision");
    }
    {
        auto cfg = config(bhv);
        cfg.set_start_in_drive(true);
        cfg.mutable_domain(1)->set_optional(true);
        Helm helm(cfg);
        Vehicle vehicle;
        double t = t0;
        bool clear = false, overridden = false;
        for (int i = 0; i < 16; ++i)
        {
            helm.handle(vehicle.navigation(t));
            if (i == 12)
                helm.handle(manual_override(true));
            const Output& out = helm.iterate(t);
            if (i < 12 && out.solution && out.solution->allstop() == ivp::interface::CLEAR)
                clear = true;
            if (i >= 12 && out.status.allstop() == ivp::interface::MANUAL_OVERRIDE &&
                out.status.state() == Status::PARK)
                overridden = true;
            t += dt;
        }
        check(clear, scenario, "no CLEAR solution with speed optional");
        check(overridden, scenario, "manual override didn't PARK with MANUAL_OVERRIDE");
    }
}

// A .bhv file with an unknown behavior parameter leaves the helm in MALCONFIG,
// with a config warning and no solution.
void malconfig()
{
    const std::string scenario = "malconfig";
    auto cfg = config(write_bhv(scenario, R"(
Behavior = BHV_ConstantHeading
{
  name    = ch
  heading = 90
  no_such_parameter = 1
}
)"));
    cfg.set_start_in_drive(true);
    Helm helm(cfg);
    check(!helm.config_warnings().empty(), scenario, "no config warning");
    Vehicle vehicle;
    helm.handle(vehicle.navigation(t0));
    const Output& out = helm.iterate(t0);
    check(out.status.state() == Status::MALCONFIG, scenario, "not MALCONFIG");
    check(!out.solution, scenario, "solution while MALCONFIG");
}

// "initialize_" (deferred) variables are posted only if nothing else has
// set them within the defer window: FOO is set from outside first and isn't
// posted; BAR isn't, and is posted.
void deferred_initialize()
{
    const std::string scenario = "deferred_initialize";
    auto cfg = config(write_bhv(scenario, R"(
initialize_ FOO = helm
initialize_ BAR = helm

Behavior = BHV_ConstantHeading
{
  name     = ch
  heading  = 90
  duration = no-time-limit
}
)"));
    cfg.mutable_domain(1)->set_optional(true);
    Helm helm(cfg);
    Vehicle vehicle;
    double t = t0;
    bool foo = false, bar = false;
    for (int i = 0; i < 12; ++i)
    {
        helm.handle(vehicle.navigation(t));
        if (i == 1)
            helm.handle(variable("FOO", "elsewhere"));
        const Output& out = helm.iterate(t);
        foo = foo || has_posting(out, "FOO", "helm", Posting::INITIALIZE);
        bar = bar || has_posting(out, "BAR", "helm", Posting::INITIALIZE);
        t += dt;
    }
    check(!foo, scenario, "posted FOO, which was already set");
    check(bar, scenario, "never posted BAR");
}

} // namespace

int main()
{
    waypoint_mission(true);
    waypoint_mission(false);
    update_through_echo();
    spawn();
    contact_alert(true);
    contact_alert(false);
    typed_outputs();
    conversion_round_trip();
    configuration();
    restart(false);
    restart(true);
    ok_skew();
    allstop();
    malconfig();
    deferred_initialize();
    return errors == 0 ? 0 : 1;
}
