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

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>

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
// function, so the helm declares NothingToDo. With park_on_allstop (the
// default) it then parks, as pHelmIvP does at the end of s1_alpha's survey.
// Without it, RETURN=true switches to ACTIVE:RETURNING and turns the vehicle
// around.
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
    if (park_on_allstop)
        check(parked && !turned, scenario, "didn't park when the survey completed");
    else
        check(turned && !parked, scenario, "never turned south in mode ACTIVE:RETURNING");
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

// Allstop reasons and park-on-allstop:
//  - no speed decision with a required speed domain gives
//    MISSING_DECISION_VARIABLES and PARK, since park_on_allstop defaults to
//    true. pHelmIvP parks during the one iteration's grace it gives a missing
//    decision, so the zero solution comes with MANUAL_OVERRIDE;
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
        bool missing = false, zeroed = false;
        for (int i = 0; i < 12; ++i)
        {
            helm.handle(vehicle.navigation(t));
            const Output& out = helm.iterate(t);
            if (out.status.allstop() == ivp::interface::MISSING_DECISION_VARIABLES &&
                out.status.missing_decision_variables_size() == 1 &&
                out.status.missing_decision_variables(0) == "speed")
                missing = true;
            if (out.solution && out.solution->allstop() != ivp::interface::CLEAR &&
                out.solution->desired().speed().value() == 0)
                zeroed = true;
            t += dt;
        }
        check(missing, scenario, "no MISSING_DECISION_VARIABLES for speed");
        check(zeroed, scenario, "no zero solution for the missing speed");
        check(helm.state() == Status::PARK, scenario, "didn't park on allstop");
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
    allstop();
    malconfig();
    deferred_initialize();
    return errors == 0 ? 0 : 1;
}
