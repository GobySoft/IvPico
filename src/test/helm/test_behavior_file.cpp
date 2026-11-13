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

// Tests for behavior files: the .bhv reader and writer, the .bhv.yml reader and
// writer, and the helm loading YAML and inline behaviors.
//
// The reader must read a .bhv file as Populator_BehaviorSet does, and the
// writer must write text the Populator reads back into the same BehaviorSet.
// Given the fixtures directory, every fixture .bhv also goes .bhv → BehaviorSet
// → .bhv → BehaviorSet (and through YAML), which must give the same set. The
// replay_<fixture>_yaml tests then check the helm decides the same from it.
//
// Prints one line per failed check and exits non-zero if there were any.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <google/protobuf/util/message_differencer.h>

#include "helm/behavior_file.h"
#include "helm/helm.h"

using google::protobuf::util::MessageDifferencer;
using ivp::interface::BehaviorSet;
using ivp::interface::HelmConfig;
using ivpico::helm::Helm;
namespace fs = std::filesystem;
namespace helm = ivpico::helm;

namespace
{
int errors = 0;

void check(bool ok, const std::string& scenario, const std::string& what)
{
    if (!ok)
    {
        std::cout << "failed: " << scenario << ": " << what << "\n";
        ++errors;
    }
}

bool equal(const BehaviorSet& a, const BehaviorSet& b, const std::string& scenario)
{
    if (MessageDifferencer::Equals(a, b))
        return true;
    std::cout << scenario << ": differ\n  " << a.ShortDebugString() << "\n  "
              << b.ShortDebugString() << "\n";
    return false;
}

std::string write_file(const std::string& name, const std::string& contents)
{
    const auto path = fs::temp_directory_path() / ("ivpico_test_" + name);
    std::ofstream(path) << contents;
    return path.string();
}

HelmConfig config()
{
    HelmConfig cfg;
    cfg.set_ownship("alpha");
    cfg.set_start_in_drive(true);
    for (const auto& [name, high, points] : std::vector<std::tuple<std::string, double, unsigned>>{
             {"course", 359, 360}, {"speed", 4, 41}})
    {
        auto& d = *cfg.add_domain();
        d.set_name(name);
        d.set_high(high);
        d.set_points(points);
    }
    return cfg;
}

// Every .bhv construct: initialize lists and deferral, modes with and without
// an else, a behavior opened on its own line, a common parameter given twice,
// comments, a continued line, and a block the Populator ignores.
const std::string constructs_bhv = R"(// comment
initialize DEPLOY = false, RETURN = false
initialize_ SPEED = 2

set MODE = ACTIVE {
  DEPLOY = true
} INACTIVE

set MODE = SURVEYING {
  MODE = ACTIVE
  RETURN = false
}

{
  ignored by the Populator
}

Behavior = BHV_Waypoint {
  name = survey   // trailing comment
  pwt = 100
  pwt = 50
  condition = MODE = SURVEYING
  endflag = RETURN = true
  speed = 3
  points = 0,0:10,10:\
           20,20
}
)";

void read_bhv_constructs()
{
    const std::string scenario = "read_bhv_constructs";
    std::vector<std::string> errs;
    helm::SourceMap map;
    const auto set = helm::read_bhv(constructs_bhv, "c.bhv", errs, &map);
    check(errs.empty(), scenario, "errors reading");

    check(set.initialize_size() == 3 && set.initialize(1).var() == "RETURN" &&
              !set.initialize(1).deferred() && set.initialize(2).deferred(),
          scenario, "initialize: " + set.ShortDebugString());
    check(set.mode_size() == 2 && set.mode(0).set() == "MODE = ACTIVE" &&
              set.mode(0).else_value() == "INACTIVE" && !set.mode(1).has_else_value() &&
              set.mode(1).condition_size() == 2,
          scenario, "modes");
    check(set.behavior_size() == 1, scenario, "one behavior");
    if (set.behavior_size() != 1)
        return;
    const auto& b = set.behavior(0);
    check(b.type() == "BHV_Waypoint" && b.name() == "survey" && b.pwt() == "100", scenario,
          "type, name and the first pwt");
    check(b.condition_size() == 1 && b.condition(0) == "MODE = SURVEYING" && b.endflag_size() == 1,
          scenario, "condition and endflag");
    check(b.param_size() == 3 && b.param(0).key() == "pwt" && b.param(0).value() == "50" &&
              b.param(2).key() == "points" && b.param(2).value() == "0,0:10,10:20,20",
          scenario, "params: " + b.ShortDebugString());

    const auto endflag = helm::locate(map, "behavior[0].endflag[0]");
    check(endflag && endflag->line == 23, scenario, "endflag's line");
    const auto points = helm::locate(map, "behavior[0].param[2]");
    check(points && points->line == 25, scenario, "a continued line is at its first line");
    const auto unknown = helm::locate(map, "behavior[0].nostarve[0]");
    check(unknown && unknown->line == 18, scenario,
          "a path without a line falls back to its behavior");
}

void read_bhv_rejects()
{
    const std::string scenario = "read_bhv_rejects";
    for (const auto& [text, line] : std::vector<std::pair<std::string, int>>{
             {"initialize A = 1\nnot a keyword\n", 2},
             {"set MODE = ACTIVE {\n  DEPLOY = = true\n}\n", 2},
             {"set MODE = A {\n  X = 1\n} TWO WORDS\n", 3},
             {"initialize A\n", 1}})
    {
        std::vector<std::string> errs;
        helm::read_bhv(text, "r.bhv", errs);
        check(errs.size() == 1 && errs[0].rfind("r.bhv:" + std::to_string(line) + ":", 0) == 0,
              scenario, "line " + std::to_string(line) + " of: " + text);
    }
}

void bhv_round_trip(const std::string& text, const std::string& scenario)
{
    std::vector<std::string> errs;
    const auto set = helm::read_bhv(text, scenario, errs);
    check(errs.empty(), scenario, "errors reading");
    std::vector<std::string> write_errs;
    const auto written = helm::bhv_text(helm::bhv_lines(set, write_errs));
    for (const auto& e : write_errs) check(false, scenario, "writing: " + e);
    const auto again = helm::read_bhv(written, scenario, errs);
    check(errs.empty(), scenario, "errors reading back");
    check(equal(set, again, scenario), scenario, ".bhv round trip");
#ifdef HAS_YAML
    const auto yaml = helm::read_yaml(helm::yaml_text(set), scenario, errs);
    for (const auto& e : errs) check(false, scenario, "reading YAML back: " + e);
    check(yaml && equal(set, *yaml, scenario), scenario, "YAML round trip");
#endif
}

void write_errors()
{
    const std::string scenario = "write_errors";
    BehaviorSet set;
    auto& init = *set.add_initialize();
    init.set_var("A");
    init.set_value("1,2");
    set.add_mode()->set_set("MODE");
    auto& b = *set.add_behavior();
    b.set_type("BHV Waypoint");
    auto* p = b.add_param();
    p->set_key("url");
    p->set_value("http://example.com");
    p = b.add_param();
    p->set_key("note");
    p->set_value("two\nlines");

    std::vector<std::string> errs;
    helm::bhv_lines(set, errs);
    for (const auto& path : {"initialize[0]", "mode[0]", "behavior[0]", "behavior[0].param[0]",
                             "behavior[0].param[1]"})
        check(std::any_of(errs.begin(), errs.end(), [&](const std::string& e)
                          { return e.rfind(std::string(path) + ":", 0) == 0; }),
              scenario, std::string("no error at ") + path);
}

void relocate()
{
    const std::string scenario = "relocate";
    helm::CombinedBehaviors combined;
    combined.line_locations = {{"a.bhv.yml", 3, 5}, {"a.bhv.yml", 7, 9}};
    check(helm::relocate_warning("Problem with line 2 in the BehaviorSet file: /tmp/x Line: [foo]",
                                 "/tmp/x", combined) == "a.bhv.yml:7:9: can't read the line [foo]",
          scenario, "a line the Populator can't read");
    check(helm::relocate_warning("/tmp/x: Line 0: speed = abc", "/tmp/x", combined) ==
              "a.bhv.yml:3:5: the behavior rejected \"speed = abc\"",
          scenario, "a parameter a behavior rejected");
    check(helm::relocate_warning("Could not read /tmp/x", "/tmp/x", combined) ==
              "Could not read behaviors",
          scenario, "another warning");
}

#ifdef HAS_YAML
void read_yaml_constructs()
{
    const std::string scenario = "read_yaml_constructs";
    const std::string yaml = R"(initialize:
  DEPLOY: false
  RETURN: false
initialize_deferred:
  SPEED: 2

modes:
  - set: MODE = ACTIVE
    condition: DEPLOY = true
    else: INACTIVE
  - set: MODE = SURVEYING
    condition: [MODE = ACTIVE, RETURN = false]

behaviors:
  - type: BHV_Waypoint
    name: survey
    pwt: 100
    condition: MODE = SURVEYING
    endflag: RETURN = true
    params:
      pwt: 50
      speed: 3
      points: 0,0:10,10:20,20
)";
    std::vector<std::string> errs;
    helm::SourceMap map;
    const auto set = helm::read_yaml(yaml, "c.bhv.yml", errs, &map);
    for (const auto& e : errs) check(false, scenario, e);
    std::vector<std::string> bhv_errs;
    const auto expected = helm::read_bhv(constructs_bhv, "c.bhv", bhv_errs);
    check(set && equal(*set, expected, scenario), scenario, "the same set as the .bhv");
    const auto speed = helm::locate(map, "behavior[0].param[1]");
    check(speed && speed->line == 22 && speed->column == 14, scenario, "speed's line and column");
}

void yaml_scalars_verbatim()
{
    const std::string scenario = "yaml_scalars_verbatim";
    const std::string yaml = R"(behaviors:
  - type: BHV_Test
    perpetual: yes
    params:
      a: 3.10
      b: on
      c: 0x1
      d: "007"
      e: 1e3
      f: ""
)";
    std::vector<std::string> errs;
    const auto set = helm::read_yaml(yaml, "v.bhv.yml", errs);
    check(set.has_value(), scenario, "read");
    if (!set)
        return;
    std::string values;
    for (const auto& p : set->behavior(0).param()) values += p.value() + "|";
    check(values == "3.10|on|0x1|007|1e3||", scenario, "values: " + values);
    check(set->behavior(0).perpetual() == "yes", scenario, "perpetual: yes");
}

void yaml_errors()
{
    const std::string scenario = "yaml_errors";
    for (const auto& [yaml, where] : std::vector<std::pair<std::string, std::string>>{
             {"behaviors:\n  - type: A\n    endflags: X = 1\n", "e.bhv.yml:3:5"},
             {"behaviors:\n  - type: A\n    pwt: [1, 2]\n", "e.bhv.yml:3:10"},
             {"behaviors:\n  - type: A\n    params:\n      speed:\n", "e.bhv.yml:4:7"},
             {"behaviors:\n  - name: A\n", "e.bhv.yml:2:5"},
             {"modes:\n  - condition: A = 1\n", "e.bhv.yml:2:5"},
             {"initialize:\n  A: 1\n  A: 2\n", "e.bhv.yml:3:3"},
             {"behavior:\n  - type: A\n", "e.bhv.yml:1:1"},
             {"behaviors: [\n", "e.bhv.yml:"},
             {"- type: A\n", "e.bhv.yml:1:1"}})
    {
        std::vector<std::string> errs;
        const auto set = helm::read_yaml(yaml, "e.bhv.yml", errs);
        check(!set && !errs.empty() && errs[0].rfind(where, 0) == 0, scenario,
              "at " + where + ": " + (errs.empty() ? "no error" : errs[0]));
    }
}

void yaml_merge()
{
    const std::string scenario = "yaml_merge";
    const std::string yaml = R"(x-waypoint: &waypoint
  pwt: 100
  params:
    speed: 3
behaviors:
  - type: BHV_Waypoint
    <<: *waypoint
    name: a
  - type: BHV_Waypoint
    <<: *waypoint
    pwt: 50
)";
    std::vector<std::string> errs;
    const auto set = helm::read_yaml(yaml, "m.bhv.yml", errs);
    check(set && set->behavior_size() == 2 && set->behavior(0).pwt() == "100" &&
              set->behavior(1).pwt() == "50" && set->behavior(1).param_size() == 1,
          scenario, "merged: " + (set ? set->ShortDebugString() : errs.front()));
}
#endif

// A behavior that decides course 90 at speed 2, given inline
BehaviorSet heading_90()
{
    BehaviorSet set;
    auto& b = *set.add_behavior();
    b.set_type("BHV_ConstantHeading");
    b.set_name("ch");
    b.set_duration("no-time-limit");
    auto* p = b.add_param();
    p->set_key("heading");
    p->set_value("90");
    auto& s = *set.add_behavior();
    s.set_type("BHV_ConstantSpeed");
    s.set_name("cs");
    s.set_duration("no-time-limit");
    p = s.add_param();
    p->set_key("speed");
    p->set_value("2");
    return set;
}

bool decides_course_90(Helm& helm)
{
    ivp::interface::Navigation nav;
    nav.set_self(true);
    nav.mutable_nav()->mutable_enu()->set_e(0);
    nav.mutable_nav()->mutable_enu()->set_n(0);
    nav.mutable_nav()->mutable_attitude()->set_heading(0);
    nav.mutable_nav()->add_speed()->set_value(0);
    const double t0 = 1.7e9;
    for (int i = 0; i < 12; ++i)
    {
        nav.mutable_nav()->set_time(static_cast<int64_t>((t0 + i * 0.25) * 1e6));
        helm.handle(nav);
        const auto& out = helm.iterate(t0 + i * 0.25);
        if (out.solution && out.solution->allstop() == ivp::interface::CLEAR)
            return std::fabs(out.solution->desired().attitude().heading() * 180 / M_PI - 90) < 1;
    }
    return false;
}

// Inline behaviors, alone and after a .bhv file; a parameter a behavior
// rejects is reported at its path in the configuration.
void helm_inline()
{
    const std::string scenario = "helm_inline";
    {
        auto cfg = config();
        *cfg.add_behaviors() = heading_90();
        Helm helm(cfg);
        check(helm.config_warnings().empty(), scenario, "config warnings");
        check(decides_course_90(helm), scenario, "inline behaviors didn't decide course 90");
    }
    {
        auto cfg = config();
        cfg.add_behavior_file(write_file("init.bhv", "initialize DEPLOY = true\n"));
        *cfg.add_behaviors() = heading_90();
        Helm helm(cfg);
        check(helm.config_warnings().empty(), scenario, "a .bhv file and inline behaviors");
    }
    {
        auto cfg = config();
        auto set = heading_90();
        auto* p = set.mutable_behavior(0)->add_param();
        p->set_key("not_a_param");
        p->set_value("1");
        *cfg.add_behaviors() = set;
        Helm helm(cfg);
        const auto& w = helm.config_warnings();
        check(!w.empty() && w[0].rfind("behaviors[0].behavior[0].param[1]: ", 0) == 0, scenario,
              "rejected parameter: " + (w.empty() ? "no warning" : w[0]));
    }
}

void helm_yaml_file()
{
    const std::string scenario = "helm_yaml_file";
    const std::string yaml = R"(behaviors:
  - type: BHV_ConstantHeading
    name: ch
    duration: no-time-limit
    params:
      heading: 90
  - type: BHV_ConstantSpeed
    name: cs
    duration: no-time-limit
    params:
      speed: 2
      not_a_param: 1
)";
    auto cfg = config();
    cfg.add_behavior_file(write_file("helm.bhv.yml", yaml));
    Helm helm(cfg);
#ifdef HAS_YAML
    const auto& w = helm.config_warnings();
    check(!w.empty() && w[0].find("ivpico_test_helm.bhv.yml:12:20: the behavior rejected") !=
                            std::string::npos,
          scenario, "rejected parameter: " + (w.empty() ? "no warning" : w[0]));

    auto fixed = yaml.substr(0, yaml.find("      not_a_param"));
    cfg.set_behavior_file(0, write_file("helm.bhv.yml", fixed));
    Helm good(cfg);
    check(good.config_warnings().empty(), scenario, "config warnings");
    check(decides_course_90(good), scenario, "YAML behaviors didn't decide course 90");
#else
    const auto& w = helm.config_warnings();
    check(!w.empty() && w[0].find("without yaml-cpp") != std::string::npos, scenario,
          "no warning that YAML needs yaml-cpp");
#endif
}

// Every fixture .bhv round-trips through the writer, and through YAML
void fixtures(const fs::path& dir)
{
    int count = 0;
    for (const auto& f : fs::recursive_directory_iterator(dir))
    {
        if (f.path().extension() != ".bhv")
            continue;
        std::ifstream in(f.path());
        std::stringstream text;
        text << in.rdbuf();
        bhv_round_trip(text.str(), "fixture " + f.path().parent_path().filename().string() + "/" +
                                       f.path().filename().string());
        ++count;
    }
    check(count > 10, "fixtures",
          "only " + std::to_string(count) + " .bhv fixtures in " + dir.string());
}
} // namespace

int main(int argc, char* argv[])
{
    read_bhv_constructs();
    read_bhv_rejects();
    bhv_round_trip(constructs_bhv, "constructs_round_trip");
    write_errors();
    relocate();
#ifdef HAS_YAML
    read_yaml_constructs();
    yaml_scalars_verbatim();
    yaml_errors();
    yaml_merge();
#endif
    helm_inline();
    helm_yaml_file();
    if (argc > 1)
        fixtures(argv[1]);
    return errors == 0 ? 0 : 1;
}
