// Replays a MOOS-IvP mission recorded under pHelmIvP (see record.sh) against
// ivpico::helm::Helm, and checks that the helm makes the same decisions and
// posts the same things.
//
// Usage: replay <fixture directory> [stream to print in full, e.g. "loiter LOITER_INDEX"]
//
// The fixture holds the vehicle's .moos file (for pHelmIvP's configuration),
// its .bhv files, and an alog trimmed to pHelmIvP's posts and the posts to
// every variable pHelmIvP registered.
//
// Open loop is enough: the helm's outputs don't change the recorded inputs.
// pHelmIvP posts IVPHELM_STATE at the start of every iteration, so those posts
// give the iteration times. Before each one, every recorded input posted at
// least `mail_latency` earlier goes to the helm as a Variable, and then the
// helm iterates at that time. Inputs pHelmIvP posted itself are left out,
// since the helm feeds its own posts back (as the MOOSDB does).
//
// Only the last value in each iteration is compared. Numbers inside values match within `number_tolerance` (relative, at least
// 0.01), since recorded inputs are rounded.
//
// What's compared, as sequences of changes, each change matched within
// `tolerance` iterations, and all but `max_mismatch` of them matching (a
// stream with fewer than 20 changes has to match exactly). Continuous values
// (distances, ETAs, track points) differ now and then, when pHelmIvP got a
// navigation update a little later or earlier than its timestamp says.
//  - IVPHELM_STATE and IVPHELM_ALLSTOP against Status;
//  - mode variables against MODE postings;
//  - initialize posts against INITIALIZE postings;
//  - each behavior's posts against its postings and behavior events;
//  - IVPHELM_LIFE_EVENT against life events;
//  - IVPHELM_BHV_ACTIVE against the ACTIVE behaviors in Status.
// DESIRED_* are compared iteration by iteration: within one step of the
// domain on all but `desired_max_mismatch` of the iterations where either
// helm decided, and never off for more than `desired_max_run` in a row.
//
// Prints a summary per stream, the first mismatches in each, and exits non-zero
// if any stream failed.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "helm/helm.h"

namespace fs = std::filesystem;
using ivp::interface::HelmConfig;
using ivp::interface::Status;
using ivpico::helm::Helm;
using ivpico::helm::Output;

namespace
{
constexpr int tolerance = 1;
constexpr double mail_latency = 0.005;
constexpr double number_tolerance = 1e-3;
constexpr double max_mismatch = 0.05;
constexpr double desired_max_mismatch = 0.01;
constexpr int desired_max_run = 3;
constexpr int report_mismatches = 5;

std::string trim(const std::string& s)
{
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
        return "";
    return s.substr(begin, s.find_last_not_of(" \t\r\n") - begin + 1);
}

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

bool is_number(const std::string& s, double* value = nullptr)
{
    if (s.empty())
        return false;
    char* end = nullptr;
    const double d = std::strtod(s.c_str(), &end);
    if (end != s.c_str() + s.size())
        return false;
    if (value)
        *value = d;
    return true;
}

struct Entry
{
    double time;
    std::string var, source, value;
};

struct Alog
{
    double logstart{0};
    std::vector<Entry> entries;
};

Alog read_alog(const fs::path& path)
{
    Alog alog;
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(
        popen(("xz -dc '" + path.string() + "'").c_str(), "r"), pclose);
    if (!pipe)
        throw std::runtime_error("can't run xz on " + path.string());
    std::string text;
    char buffer[65536];
    for (size_t n; (n = fread(buffer, 1, sizeof(buffer), pipe.get())) > 0;) text.append(buffer, n);

    std::istringstream lines(text);
    for (std::string line; std::getline(lines, line);)
    {
        if (line.rfind("%%", 0) == 0)
        {
            std::istringstream header(line.substr(2));
            std::string key;
            header >> key;
            if (key == "LOGSTART")
                header >> alog.logstart;
            continue;
        }
        std::istringstream fields(line);
        Entry e;
        if (!(fields >> e.time >> e.var >> e.source))
            continue;
        std::getline(fields, e.value);
        e.value = trim(e.value);
        alog.entries.push_back(e);
    }
    std::stable_sort(alog.entries.begin(), alog.entries.end(),
                     [](const Entry& a, const Entry& b) { return a.time < b.time; });
    return alog;
}

// pHelmIvP's block of the .moos file, as HelmConfig
HelmConfig read_config(const fs::path& moos_file, const fs::path& dir)
{
    HelmConfig cfg;
    std::ifstream in(moos_file);
    bool in_block = false, in_helm = false;
    std::vector<std::string> helm_lines;
    for (std::string line; std::getline(in, line);)
    {
        line = trim(line.substr(0, line.find("//")));
        if (line.empty())
            continue;
        const auto eq = line.find('=');
        const std::string key = lower(trim(line.substr(0, eq)));
        const std::string value = eq == std::string::npos ? "" : trim(line.substr(eq + 1));

        if (!in_block && key == "community")
            cfg.set_ownship(value);
        if (key == "processconfig")
        {
            in_helm = value == "pHelmIvP";
            continue;
        }
        if (line == "{")
        {
            in_block = true;
            continue;
        }
        if (line == "}")
        {
            in_block = in_helm = false;
            continue;
        }
        if (in_block && in_helm)
            helm_lines.push_back(line);
    }

    // MOOS's mission reader hands pHelmIvP its configuration in reverse order,
    // which sets the order of the domain variables
    for (auto it = helm_lines.rbegin(); it != helm_lines.rend(); ++it)
    {
        const std::string& line = *it;
        const auto eq = line.find('=');
        const std::string key = lower(trim(line.substr(0, eq)));
        const std::string value = eq == std::string::npos ? "" : trim(line.substr(eq + 1));
        const bool truth = lower(value) == "true";
        if (key == "behaviors")
            cfg.add_behavior_file((dir / fs::path(value).filename()).string());
        else if (key == "domain")
        {
            std::string spec = value;
            std::replace(spec.begin(), spec.end(), ',', ':');
            std::vector<std::string> parts;
            std::istringstream ps(spec);
            for (std::string p; std::getline(ps, p, ':');) parts.push_back(trim(p));
            auto* d = cfg.add_domain();
            d->set_name(parts.at(0));
            d->set_low(std::stod(parts.at(1)));
            d->set_high(std::stod(parts.at(2)));
            if (parts.at(3).rfind("delta=", 0) == 0)
                d->set_delta(std::stod(parts.at(3).substr(6)));
            else
                d->set_points(std::stoul(parts.at(3)));
            if (parts.size() > 4 && lower(parts[4]) == "optional")
                d->set_optional(true);
        }
        else if (key == "park_on_allstop")
            cfg.set_park_on_allstop(truth);
        else if (key == "allow_park")
            cfg.set_allow_park(truth);
        else if (key == "start_in_drive" || key == "start_indrive" || key == "start_engaged" ||
                 key == "active_start")
            cfg.set_start_in_drive(truth);
        else if (key == "goals_mandatory")
            cfg.set_goals_mandatory(truth);
        else if (key == "seed_random")
            cfg.set_seed_random(truth);
        else if (key == "helm_alias")
            cfg.set_helm_alias(value);
        else if (key == "nav_grace")
            cfg.set_nav_grace(std::stod(value));
        else if (key == "contact_max_age")
            cfg.set_contact_max_age(std::stod(value));
        else if (key == "contact_max_count")
            cfg.set_contact_max_count(std::stoul(value));
        else if (key == "ok_skew")
            cfg.set_ok_skew(lower(value) == "any" ? -1 : std::stod(value));
        else if (key == "pmgen")
        {
            auto* pm = cfg.mutable_platform_model();
            std::istringstream ps(value);
            for (std::string p; std::getline(ps, p, '#');)
            {
                const auto peq = p.find('=');
                const std::string pk = lower(trim(p.substr(0, peq)));
                const std::string pv = trim(p.substr(peq + 1));
                if (pk == "alg")
                    pm->set_algorithm(lower(pv) == "dubins" ? HelmConfig::PlatformModel::DUBINS
                                                            : HelmConfig::PlatformModel::HOLONOMIC);
                else if (pk == "radius" || pk == "rad")
                    pm->set_radius(std::stod(pv));
                else if (pk == "spoke_degs" || pk == "degs")
                    pm->set_spoke_degs(std::stod(pv));
            }
        }
        else
            std::cout << "  (pHelmIvP parameter not replayed: " << line << ")\n";
    }
    return cfg;
}

struct Change
{
    int tick;
    std::string value;
};

using Streams = std::map<std::string, std::vector<Change>>;

// Keeps the last value in each iteration, and only changes
void add_change(Streams& streams, const std::string& stream, int tick, const std::string& value)
{
    auto& changes = streams[stream];
    if (!changes.empty() && changes.back().tick == tick)
    {
        changes.back().value = value;
        if (changes.size() > 1 && changes[changes.size() - 2].value == value)
            changes.pop_back();
        return;
    }
    if (changes.empty() || changes.back().value != value)
        changes.push_back({tick, value});
}

// Splits "x=1.5,y=-2" into its text ("x=,y=") and its numbers (1.5, -2)
std::pair<std::string, std::vector<double>> split_numbers(const std::string& s)
{
    std::pair<std::string, std::vector<double>> parts;
    for (size_t i = 0; i < s.size();)
    {
        const bool starts_number = std::isdigit(static_cast<unsigned char>(s[i])) ||
                                   ((s[i] == '-' || s[i] == '.') && i + 1 < s.size() &&
                                    std::isdigit(static_cast<unsigned char>(s[i + 1])));
        if (starts_number)
        {
            char* end = nullptr;
            parts.second.push_back(std::strtod(s.c_str() + i, &end));
            i = end - s.c_str();
        }
        else
            parts.first += s[i++];
    }
    return parts;
}

bool same_value(const std::string& a, const std::string& b)
{
    if (a == b)
        return true;
    const auto pa = split_numbers(a), pb = split_numbers(b);
    if (pa.first != pb.first || pa.second.size() != pb.second.size())
        return false;
    for (size_t i = 0; i < pa.second.size(); ++i)
        if (std::fabs(pa.second[i] - pb.second[i]) >
            std::max(0.01, number_tolerance * std::fabs(pa.second[i])))
            return false;
    return true;
}

std::string number(double d)
{
    std::ostringstream os;
    os << d;
    return os.str();
}

std::string allstop_string(const Status& status)
{
    switch (status.allstop())
    {
        case ivp::interface::CLEAR: return "clear";
        case ivp::interface::MANUAL_OVERRIDE: return "ManualOverride";
        case ivp::interface::BEHAVIOR_ERROR: return "BehaviorError";
        case ivp::interface::NOTHING_TO_DO: return "NothingToDo";
        case ivp::interface::NO_GOAL_BEHAVIOR: return "NoGoalBehavior";
        case ivp::interface::DISABLED_BY_STANDBY_HELM: return "DisabledByStandbyHelm";
        case ivp::interface::MISSING_DECISION_VARIABLES:
        {
            std::string s = "MissingDecVars:";
            for (int i = 0; i < status.missing_decision_variables_size(); ++i)
                s += (i ? "," : "") + status.missing_decision_variables(i);
            return s;
        }
        default: return "";
    }
}

std::string behavior_event_var(ivp::interface::BehaviorEvent::Kind kind)
{
    using ivp::interface::BehaviorEvent;
    switch (kind)
    {
        case BehaviorEvent::WARNING:
        case BehaviorEvent::WARNING_RETRACTED: return "BHV_WARNING";
        case BehaviorEvent::ERROR: return "BHV_ERROR";
        case BehaviorEvent::EVENT: return "BHV_EVENT";
        case BehaviorEvent::CONFIG_WARNING: return "BHV_CONFIG_WARNING";
        case BehaviorEvent::BAD_CONFIG: return "BHV_BAD_CONFIG";
        case BehaviorEvent::STATUS: return "BHV_STATUS";
        case BehaviorEvent::SETTINGS: return "BHV_SETTINGS";
        default: return "BHV_UNKNOWN";
    }
}

// "event=spawn,bname=ch_one,..." to "spawn ch_one"
std::string life_event(const std::string& spec)
{
    std::string event, bname;
    std::istringstream fields(spec);
    for (std::string f; std::getline(fields, f, ',');)
    {
        const auto eq = f.find('=');
        const std::string key = trim(f.substr(0, eq)), value = trim(f.substr(eq + 1));
        if (key == "event")
            event = value;
        else if (key == "bname")
            bname = value;
    }
    return event + " " + bname;
}

// Compares two sequences of changes, matching values in order and ticks within tolerance
bool compare(const std::string& stream, const std::vector<Change>& moos,
             const std::vector<Change>& ivpico)
{
    std::vector<std::string> problems;
    size_t i = 0, j = 0;
    while (i < moos.size() && j < ivpico.size())
    {
        if (same_value(moos[i].value, ivpico[j].value) &&
            std::abs(moos[i].tick - ivpico[j].tick) <= tolerance)
        {
            ++i, ++j;
            continue;
        }
        problems.push_back("pHelmIvP iteration " + std::to_string(moos[i].tick) + ": \"" +
                           moos[i].value + "\", IvPico iteration " +
                           std::to_string(ivpico[j].tick) + ": \"" + ivpico[j].value + "\"");
        // a different value on the same iteration is one mismatch; otherwise
        // resynchronize on whichever comes first
        if (moos[i].tick == ivpico[j].tick)
            ++i, ++j;
        else if (moos[i].tick < ivpico[j].tick)
            ++i;
        else
            ++j;
    }
    for (; i < moos.size(); ++i)
        problems.push_back("pHelmIvP iteration " + std::to_string(moos[i].tick) + ": \"" +
                           moos[i].value + "\", missing from IvPico");
    for (; j < ivpico.size(); ++j)
        problems.push_back("IvPico iteration " + std::to_string(ivpico[j].tick) + ": \"" +
                           ivpico[j].value + "\", missing from pHelmIvP");

    const bool ok = problems.size() <= max_mismatch * std::max(moos.size(), ivpico.size());
    std::cout << (ok ? "  ok    " : "  FAIL  ") << stream << " (" << moos.size() << " changes, "
              << problems.size() << " off)\n";
    for (size_t k = 0; k < problems.size() && k < report_mismatches; ++k)
        std::cout << "          " << problems[k] << "\n";
    if (problems.size() > report_mismatches)
        std::cout << "          ... and " << problems.size() - report_mismatches << " more\n";
    return ok;
}

bool compare_desired(const std::string& var, double step, bool wrap,
                     const std::map<int, double>& moos, const std::map<int, double>& ivpico)
{
    std::map<int, std::pair<std::optional<double>, std::optional<double>>> ticks;
    for (const auto& [k, v] : moos) ticks[k].first = v;
    for (const auto& [k, v] : ivpico) ticks[k].second = v;

    int mismatches = 0, run = 0, longest_run = 0;
    std::vector<std::string> examples;
    for (const auto& [k, values] : ticks)
    {
        bool ok = values.first && values.second;
        if (ok)
        {
            double diff = std::fabs(*values.first - *values.second);
            if (wrap)
                diff = std::min(diff, 360 - diff);
            ok = diff <= step + 1e-6;
        }
        if (ok)
        {
            run = 0;
            continue;
        }
        ++mismatches;
        longest_run = std::max(longest_run, ++run);
        if (examples.size() < report_mismatches)
            examples.push_back("iteration " + std::to_string(k) + ": pHelmIvP " +
                               (values.first ? number(*values.first) : std::string("none")) +
                               ", IvPico " +
                               (values.second ? number(*values.second) : std::string("none")));
    }
    const bool ok = ticks.empty() || (mismatches <= desired_max_mismatch * ticks.size() &&
                                      longest_run <= desired_max_run);
    std::cout << (ok ? "  ok    " : "  FAIL  ") << var << " (" << ticks.size() << " iterations, "
              << mismatches << " off, longest run " << longest_run << ")\n";
    for (const auto& e : examples) std::cout << "          " << e << "\n";
    return ok;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc != 2 && argc != 3)
    {
        std::cerr << "usage: replay <fixture directory> [stream to print in full]\n";
        return 2;
    }
    const fs::path dir = fs::path(argv[1]).lexically_normal();
    fs::path moos_file, alog_file;
    for (const auto& f : fs::directory_iterator(dir))
    {
        if (f.path().extension() == ".moos")
            moos_file = f.path();
        else if (f.path().string().size() > 8 &&
                 f.path().string().substr(f.path().string().size() - 8) == ".alog.xz")
            alog_file = f.path();
    }
    std::cout << "Replaying " << (dir.has_filename() ? dir : dir.parent_path()).filename().string()
              << "\n";

    const HelmConfig cfg = read_config(moos_file, dir);
    const Alog alog = read_alog(alog_file);

    auto is_helm = [](const Entry& e) { return e.source.rfind("pHelmIvP", 0) == 0; };
    auto aux = [](const Entry& e)
    { return e.source.size() > 9 ? e.source.substr(9) : std::string(); };

    std::vector<double> ticks;
    for (const auto& e : alog.entries)
        if (is_helm(e) && e.var == "IVPHELM_STATE")
            ticks.push_back(e.time);
    auto tick_of = [&](double time)
    {
        return std::max(0, static_cast<int>(std::upper_bound(ticks.begin(), ticks.end(), time) -
                                            ticks.begin()) -
                               1);
    };

    // pHelmIvP's side
    Streams moos;
    std::map<std::string, std::map<int, double>> moos_desired;
    for (const auto& e : alog.entries)
    {
        if (!is_helm(e))
            continue;
        const int k = tick_of(e.time);
        const std::string a = aux(e);
        if (e.var == "IVPHELM_STATE")
            add_change(moos, "IVPHELM_STATE", k, e.value);
        else if (e.var == "IVPHELM_ALLSTOP")
            add_change(moos, "IVPHELM_ALLSTOP", k, e.value);
        else if (e.var == "IVPHELM_LIFE_EVENT")
            moos["IVPHELM_LIFE_EVENT"].push_back({k, life_event(e.value)});
        else if (e.var == "IVPHELM_BHV_ACTIVE")
            add_change(moos, "IVPHELM_BHV_ACTIVE", k, e.value);
        else if (e.var.rfind("DESIRED_", 0) == 0 && a.empty())
        {
            double d;
            if (is_number(e.value, &d))
                moos_desired[e.var][k] = d;
        }
        else if (a == "HELM_MODE")
            add_change(moos, "mode " + e.var, k, e.value);
        else if (a == "HELM_VAR_INIT")
            add_change(moos, "initialize " + e.var, k, e.value);
        else if (a.find(':') != std::string::npos && e.var != "BHV_IPF")
            add_change(moos, a.substr(a.find(':') + 1) + " " + e.var, k, e.value);
    }

    // IvPico's side
    Helm helm(cfg);
    for (const auto& w : helm.config_warnings()) std::cout << "  config warning: " << w << "\n";

    Streams ivpico;
    std::map<std::string, std::map<int, double>> ivpico_desired;
    size_t next_input = 0;
    for (size_t k = 0; k < ticks.size(); ++k)
    {
        for (; next_input < alog.entries.size() &&
               alog.entries[next_input].time <= ticks[k] - mail_latency;
             ++next_input)
        {
            const Entry& e = alog.entries[next_input];
            if (is_helm(e))
                continue;
            ivp::interface::Variable v;
            v.set_time(static_cast<int64_t>((alog.logstart + e.time) * 1e6));
            v.set_key(e.var);
            double d;
            if (is_number(e.value, &d))
                v.set_dval(d);
            else
                v.set_sval(e.value);
            helm.handle(v);
        }

        const Output& out = helm.iterate(alog.logstart + ticks[k]);
        const int tick = static_cast<int>(k);
        add_change(ivpico, "IVPHELM_STATE", tick, Status::State_Name(out.status.state()));
        if (out.status.allstop() != ivp::interface::ALLSTOP_UNKNOWN)
            add_change(ivpico, "IVPHELM_ALLSTOP", tick, allstop_string(out.status));

        std::string active;
        for (const auto& b : out.status.behavior())
            if (b.run_state() == Status::Behavior::ACTIVE)
                active += (active.empty() ? "" : ",") + b.name();
        // pHelmIvP posts the list only when it changes, starting from none
        if (!ivpico["IVPHELM_BHV_ACTIVE"].empty() || !active.empty())
            add_change(ivpico, "IVPHELM_BHV_ACTIVE", tick, active);

        for (const auto& p : out.postings)
        {
            const std::string value =
                p.value_case() == ivp::interface::Posting::kSval ? p.sval() : number(p.dval());
            switch (p.origin())
            {
                case ivp::interface::Posting::MODE:
                    add_change(ivpico, "mode " + p.key(), tick, value);
                    break;
                case ivp::interface::Posting::INITIALIZE:
                    add_change(ivpico, "initialize " + p.key(), tick, value);
                    break;
                case ivp::interface::Posting::BEHAVIOR:
                    add_change(ivpico, p.behavior() + " " + p.key(), tick, value);
                    break;
                default: break;
            }
        }
        for (const auto& e : out.behavior_events)
            if (!e.behavior().empty())
                add_change(ivpico, e.behavior() + " " + behavior_event_var(e.kind()), tick,
                           e.text());
        for (const auto& e : out.life_events)
        {
            const std::string type = lower(ivp::interface::LifeEvent::Type_Name(e.type()));
            ivpico["IVPHELM_LIFE_EVENT"].push_back({tick, type + " " + e.behavior()});
        }
        if (out.solution)
        {
            const auto& d = out.solution->desired();
            if (d.has_attitude() && d.attitude().has_heading())
            {
                double h = std::fmod(d.attitude().heading() * 180 / M_PI, 360);
                ivpico_desired["DESIRED_HEADING"][tick] = h < 0 ? h + 360 : h;
            }
            if (d.has_speed() && d.speed().has_value())
                ivpico_desired["DESIRED_SPEED"][tick] = d.speed().value();
            if (d.has_depth())
                ivpico_desired["DESIRED_DEPTH"][tick] = d.depth();
            if (d.has_altitude())
                ivpico_desired["DESIRED_ALTITUDE"][tick] = d.altitude();
            if (d.has_attitude() && d.attitude().has_pitch())
                ivpico_desired["DESIRED_PITCH"][tick] = d.attitude().pitch() * 180 / M_PI;
            for (const auto& c : d.custom()) ivpico_desired[c.name()][tick] = c.value();
        }
    }
    std::cout << "  " << ticks.size() << " iterations\n";

    bool ok = true;
    std::map<std::string, bool> names;
    for (const auto& [name, changes] : moos) names[name] = true;
    for (const auto& [name, changes] : ivpico) names[name] = true;
    for (const auto& [name, unused] : names) ok = compare(name, moos[name], ivpico[name]) && ok;

    if (argc == 3)
    {
        std::cout << "pHelmIvP " << argv[2] << ":\n";
        for (const auto& c : moos[argv[2]]) std::cout << "  " << c.tick << " " << c.value << "\n";
        std::cout << "IvPico " << argv[2] << ":\n";
        for (const auto& c : ivpico[argv[2]]) std::cout << "  " << c.tick << " " << c.value << "\n";
    }

    std::map<std::string, bool> desired_vars;
    for (const auto& [var, values] : moos_desired) desired_vars[var] = true;
    for (const auto& [var, values] : ivpico_desired) desired_vars[var] = true;
    for (const auto& [var, unused] : desired_vars)
    {
        double step = 0;
        bool wrap = false;
        for (const auto& d : cfg.domain())
        {
            std::string name = "DESIRED_";
            for (char c : d.name()) name += static_cast<char>(std::toupper(c));
            if (name == "DESIRED_COURSE")
                name = "DESIRED_HEADING", wrap = true;
            if (name == var)
                step = d.resolution_case() == HelmConfig::Domain::kDelta
                           ? d.delta()
                           : (d.high() - d.low()) / std::max(1u, d.points() - 1);
        }
        ok = compare_desired(var, step, wrap, moos_desired[var], ivpico_desired[var]) && ok;
    }

    std::cout << (ok ? "PASSED" : "FAILED") << "\n";
    return ok ? 0 : 1;
}
