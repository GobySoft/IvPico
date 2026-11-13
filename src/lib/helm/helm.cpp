// Copyright 2026:
//   GobySoft, LLC
// Ports logic from MOOS-IvP's pHelmIvP (HelmIvP.cpp), by Michael Benjamin,
// Dept of Mechanical Engineering, MIT.
//
// This file is part of IvPico.
//
// IvPico is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// IvPico is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with IvPico. If not, see <https://www.gnu.org/licenses/>.

#include "helm.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <map>
#include <set>
#include <variant>

#include <unistd.h>

#include "BehaviorSet.h"
#include "BuildUtils.h"
#include "ContactLedger.h"
#include "HelmEngine.h"
#include "HelmReport.h"
#include "InfoBuffer.h"
#include "IvPBehavior.h"
#include "IvPDomain.h"
#include "LedgerSnap.h"
#include "LifeEvent.h"
#include "MBUtils.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "PlatModelGenerator.h"
#include "Populator_BehaviorSet.h"
#include "VarDataPair.h"

#include "behavior_file.h"
#include "contact_manager.h"
#include "conversions.h"

using ivp::interface::AllStop;
using ivp::interface::BehaviorEvent;
using ivp::interface::Command;
using ivp::interface::HelmConfig;
using ivp::interface::Navigation;
using ivp::interface::Posting;
using ivp::interface::Status;
using ivp::interface::Variable;

namespace ivpico
{
namespace helm
{
namespace
{
struct Value
{
    bool is_string{false};
    double d{0};
    std::string s;
    // posted by the helm's "initialize"
    bool init{false};
};

Value make_value(const VarDataPair& pair)
{
    Value v;
    v.is_string = pair.is_string();
    v.d = pair.get_ddata();
    v.s = pair.get_sdata();
    return v;
}

void set_value(Posting& posting, const Value& v)
{
    if (v.is_string)
        posting.set_sval(v.s);
    else
        posting.set_dval(v.d);
}

std::optional<BehaviorEvent::Kind> behavior_event_kind(const std::string& var,
                                                       const std::string& key)
{
    if (var == "BHV_WARNING")
        return strEnds(key, "retract") ? BehaviorEvent::WARNING_RETRACTED : BehaviorEvent::WARNING;
    if (var == "BHV_ERROR")
        return BehaviorEvent::ERROR;
    if (var == "BHV_EVENT")
        return BehaviorEvent::EVENT;
    if (var == "BHV_CONFIG_WARNING")
        return BehaviorEvent::CONFIG_WARNING;
    if (var == "BHV_BAD_CONFIG")
        return BehaviorEvent::BAD_CONFIG;
    if (var == "BHV_STATUS")
        return BehaviorEvent::STATUS;
    if (var == "BHV_SETTINGS")
        return BehaviorEvent::SETTINGS;
    return std::nullopt;
}

AllStop allstop_enum(const std::string& msg)
{
    if (msg == "clear")
        return ivp::interface::CLEAR;
    if (msg == "ManualOverride")
        return ivp::interface::MANUAL_OVERRIDE;
    if (msg == "BehaviorError")
        return ivp::interface::BEHAVIOR_ERROR;
    if (msg == "NothingToDo")
        return ivp::interface::NOTHING_TO_DO;
    if (msg == "NoGoalBehavior")
        return ivp::interface::NO_GOAL_BEHAVIOR;
    if (strBegins(msg, "MissingDecVars"))
        return ivp::interface::MISSING_DECISION_VARIABLES;
    if (msg == "DisabledByStandbyHelm")
        return ivp::interface::DISABLED_BY_STANDBY_HELM;
    return ivp::interface::ALLSTOP_UNKNOWN;
}

std::string desired_variable(const std::string& domain_var)
{
    std::string var = "DESIRED_" + toupper(domain_var);
    return var == "DESIRED_COURSE" ? "DESIRED_HEADING" : var;
}

int64_t micros(double seconds) { return static_cast<int64_t>(std::llround(seconds * 1e6)); }

double seconds(int64_t micros, double fallback)
{
    return micros == 0 ? fallback : static_cast<double>(micros) * 1e-6;
}

} // namespace

class Helm::Impl
{
  public:
    Impl(const HelmConfig& cfg, std::optional<Geodesy> geodesy);
    ~Impl() { delete bhv_set_; }

    using Input = std::variant<Navigation, Variable, Command>;
    void queue(Input input) { pending_.push_back(std::move(input)); }
    const Output& iterate(double time);

    Status::State state() const { return status_; }
    const std::vector<std::string>& config_warnings() const { return config_warnings_; }

  private:
    void configure();
    void build_behaviors();
    void start(double time);
    void restart(const std::string& type);
    void update_configuration();
    bool skewed(const std::string& var, double msg_time);

    // mail
    void apply_pending(double time);
    void apply(const Navigation& nav, double time);
    void apply(const Command& command, double time);
    void apply_variable(const std::string& var, const Value& value, double msg_time);
    void set_manual_override(bool on);
    bool process_node_record(NodeRecord record);
    void process_node_report_local(const NodeRecord& record);
    void register_variable(const std::string& var);
    void register_variables();

    // iterate
    void update_status();
    bool enabled() const { return status_ == Status::PARK || status_ == Status::DRIVE; }
    bool hold_for_nav_solution();
    void post_initial_variables(bool deferred);
    void post_helm_start_messages();
    void update_ledger_snap();
    void run_contact_manager();
    void update_plat_model();
    void post_mode_messages();
    void post_behavior_messages();
    void post_life_events();
    void post_allstop(const std::string& msg);
    void post_decisions();
    void fill_status();

    bool detect_change(const std::string& key, const std::string& value);
    bool detect_change(const std::string& key, double value);
    void notify(const std::string& var, const Value& value);
    void notify(const std::string& var, double d) { notify(var, Value{false, d, {}}); }
    void notify(const std::string& var, const std::string& s) { notify(var, Value{true, 0, s}); }
    Posting& add_posting(Posting::Origin origin, const std::string& behavior,
                         const std::string& var, const Value& value);
    void add_behavior_event(BehaviorEvent::Kind kind, const std::string& behavior,
                            const std::string& text);

    HelmConfig cfg_;
    std::optional<Geodesy> geodesy_;
    std::vector<std::string> config_warnings_;

    IvPDomain domain_;
    std::set<std::string> optional_vars_;
    InfoBuffer info_buffer_;
    LedgerSnap ledger_snap_;
    ContactLedger ledger_;
    std::unique_ptr<ContactManager> contact_manager_;
    PlatModelGenerator pmgen_;
    std::unique_ptr<HelmEngine> engine_;
    BehaviorSet* bhv_set_{nullptr};
    HelmReport report_;

    std::string ownship_;
    Status::State status_{Status::PARK};
    bool has_control_{false};
    bool allow_park_{true};
    bool park_on_allstop_{true};
    double nav_grace_{5};
    double defer_window_{1};
    double ok_skew_{60};
    unsigned warning_count_{0};
    // percent of a CPU the helm used between iterations
    double cpu_load_{0};
    std::clock_t last_cpu_{0};
    std::chrono::steady_clock::time_point last_wall_;
    void update_cpu_load();
    bool restart_post_pending_{false};
    std::string configuration_;

    bool started_{false};
    double curr_time_{0};
    double start_time_{0};
    bool nav_started_{false};
    bool init_vars_done_{false};
    bool helm_start_posted_{false};
    bool refresh_pending_{false};
    double refresh_time_{0};
    const double refresh_interval_{2};

    std::string allstop_msg_;
    std::string allstop_reason_;
    unsigned no_decisions_{0};
    std::string comms_policy_{"open"};
    unsigned iteration_{0};
    std::vector<std::string> missing_decision_vars_;

    std::map<std::string, std::string> outgoing_strings_;
    std::map<std::string, double> outgoing_doubles_;
    std::set<std::string> registered_;

    // What pHelmIvP would have posted to the MOOSDB this iteration, which the
    // MOOSDB delivers back on the next one for the variables the helm registered
    std::vector<std::pair<std::string, Value>> posted_;
    double posted_time_{0};
    std::vector<Input> pending_;
    Output output_;
};

Helm::Impl::Impl(const HelmConfig& cfg, std::optional<Geodesy> geodesy)
    : cfg_(cfg), geodesy_(std::move(geodesy))
{
    configure();
}

void Helm::Impl::configure()
{
    ownship_ = cfg_.ownship();
    has_control_ = cfg_.start_in_drive();
    if (cfg_.has_allow_park())
        allow_park_ = cfg_.allow_park();
    if (cfg_.has_park_on_allstop())
        park_on_allstop_ = cfg_.park_on_allstop();
    if (cfg_.has_nav_grace())
        nav_grace_ = cfg_.nav_grace();
    if (cfg_.has_defer_window())
        defer_window_ = cfg_.defer_window();

    if (geodesy_)
        ledger_.setGeodesy(geodesy_->to_local, geodesy_->to_global);
    double contact_max_age = cfg_.has_contact_max_age() ? cfg_.contact_max_age() : 45;
    if (contact_max_age < 10 || contact_max_age > 1800)
        config_warnings_.push_back("contact_max_age must be from 10 to 1800 seconds");
    ledger_.setStaleThresh(vclip(contact_max_age, 10, 1800));
    if (cfg_.has_contact_max_count())
        ledger_.setMaxContacts(cfg_.contact_max_count());

    if (!cfg_.has_contact_alerts() || cfg_.contact_alerts())
    {
        contact_manager_ =
            std::make_unique<ContactManager>(cfg_.contact_manager(), ownship_, config_warnings_);
        if (geodesy_)
            contact_manager_->ledger().setGeodesy(geodesy_->to_local, geodesy_->to_global);
    }

    const auto& pm = cfg_.platform_model();
    std::string pmgen =
        pm.algorithm() == HelmConfig::PlatformModel::DUBINS ? "alg=dubins" : "alg=holo";
    if (pm.has_radius())
        pmgen += "#radius=" + doubleToStringX(pm.radius());
    if (pm.has_spoke_degs())
        pmgen += "#spoke_degs=" + doubleToStringX(pm.spoke_degs());
    if (!pmgen_.setParams(pmgen))
        config_warnings_.push_back("Bad platform_model: " + pm.ShortDebugString());

    if (!cfg_.has_seed_random() || cfg_.seed_random())
        srand(static_cast<unsigned>(time(nullptr) * 31 + getpid()));

    if (cfg_.has_ok_skew())
        ok_skew_ = cfg_.ok_skew();

    build_behaviors();
}

// The behaviors and the domain, at startup and on each restart
void Helm::Impl::build_behaviors()
{
    delete bhv_set_;
    bhv_set_ = nullptr;
    engine_.reset();
    domain_ = IvPDomain();
    optional_vars_.clear();

    for (const auto& d : cfg_.domain())
    {
        unsigned points = d.points();
        const double range = d.high() - d.low();
        if (d.resolution_case() == HelmConfig::Domain::kDelta && d.delta() > 0 && range > 0 &&
            d.delta() <= range)
            points = static_cast<unsigned>(range / d.delta() + 1);
        if (d.high() < d.low() || (range == 0 && points != 1) ||
            !domain_.addDomain(d.name().c_str(), d.low(), d.high(), points))
            config_warnings_.push_back("Bad domain: " + d.ShortDebugString());
        if (d.optional())
            optional_vars_.insert(d.name());
    }

    if (!config_warnings_.empty())
        return;

    engine_ = std::make_unique<HelmEngine>(domain_, &info_buffer_, &ledger_snap_);
    Populator_BehaviorSet populator(domain_, &info_buffer_, &ledger_snap_);
    populator.setBHVDirNotFoundOK(cfg_.behavior_dir_not_found_ok());
    populator.setOwnship(ownship_);
    for (const auto& dir : cfg_.behavior_dir()) populator.addBehaviorDir(dir);
    if (!needs_combining(cfg_))
    {
        bhv_set_ = populator.populate(
            std::set<std::string>(cfg_.behavior_file().begin(), cfg_.behavior_file().end()));
        for (const auto& w : populator.getConfigWarnings()) config_warnings_.push_back(w);
    }
    else
    {
        const CombinedBehaviors combined = combine_behaviors(cfg_, config_warnings_);
        if (!config_warnings_.empty())
            return;
        // the Populator reads only files
        std::string path =
            (std::filesystem::temp_directory_path() / "ivpico_behaviors_XXXXXX").string();
        const int fd = mkstemp(path.data());
        if (fd < 0 || write(fd, combined.text.data(), combined.text.size()) !=
                          static_cast<ssize_t>(combined.text.size()))
        {
            config_warnings_.push_back("Couldn't write the combined behaviors to " + path);
            if (fd >= 0)
                close(fd);
            return;
        }
        close(fd);
        bhv_set_ = populator.populate(path);
        std::filesystem::remove(path);
        for (const auto& w : populator.getConfigWarnings())
            config_warnings_.push_back(relocate_warning(w, path, combined));
    }

    if (!bhv_set_)
    {
        if (config_warnings_.empty())
            config_warnings_.push_back("No behaviors from the behavior files");
        return;
    }
    if (!config_warnings_.empty())
        return;

    bhv_set_->setReportIPF(cfg_.report_objective_functions());
    engine_->setBehaviorSet(bhv_set_);
    for (unsigned i = 0; i < bhv_set_->size(); ++i)
    {
        // pHelmIvP sets "us" itself, since only it knows the community name
        bhv_set_->getBehavior(i)->IvPBehavior::setParam("us", ownship_);
        bhv_set_->getBehavior(i)->onSetParamComplete();
    }
}

void Helm::Impl::start(double time)
{
    started_ = true;
    start_time_ = time;
    info_buffer_.setCurrTime(time);
    info_buffer_.setStartTime(time);
    ledger_.setCurrTimeUTC(time);

    if (!config_warnings_.empty())
    {
        status_ = Status::MALCONFIG;
        has_control_ = false;
        return;
    }

    post_initial_variables(false);
    post_helm_start_messages();
    register_variables();

    notify("IVPHELM_DOMAIN", domainToString(domain_));
    notify("IVPHELM_MODESET", bhv_set_->getModeSetDefinition());
    if (!cfg_.helm_alias().empty())
        notify("IVPHELM_ALIAS", cfg_.helm_alias());
    update_configuration();
}

// HelmIvP's RESTART_HELM: type1 rebuilds the behaviors, type2 also posts the
// "initialize" variables again. Unlike pHelmIvP, the behavior files are kept.
void Helm::Impl::restart(const std::string& type)
{
    if (type != "type1" && type != "type2")
        return;

    outgoing_strings_.clear();
    outgoing_doubles_.clear();
    init_vars_done_ = false;
    config_warnings_.clear();
    build_behaviors();
    if (!config_warnings_.empty())
    {
        status_ = Status::MALCONFIG;
        has_control_ = false;
        return;
    }

    if (type == "type2")
        post_initial_variables(false);
    register_variables();
    notify("IVPHELM_DOMAIN", domainToString(domain_));
    notify("IVPHELM_MODESET", bhv_set_->getModeSetDefinition());
    if (!cfg_.helm_alias().empty())
        notify("IVPHELM_ALIAS", cfg_.helm_alias());
    restart_post_pending_ = true;
    add_behavior_event(BehaviorEvent::WARNING, "", "The helm has been restarted");
    update_configuration();
}

// IVPHELM_DOMAIN, IVPHELM_MODESET, IVPHELM_STATEVARS, IVPHELM_UPDATEVARS and
// IVPHELM_REGISTER as one Configuration, output when any of them changes
void Helm::Impl::update_configuration()
{
    if (!bhv_set_)
        return;

    ivp::interface::Configuration c;
    c.set_alias(cfg_.helm_alias());
    for (unsigned i = 0; i < domain_.size(); ++i)
    {
        auto& d = *c.add_domain();
        d.set_name(domain_.getVarName(i));
        d.set_low(domain_.getVarLow(i));
        d.set_high(domain_.getVarHigh(i));
        d.set_points(domain_.getVarPoints(i));
        d.set_optional_decision(optional_vars_.count(d.name()) > 0);
    }
    c.set_mode_set(bhv_set_->getModeSetDefinition());
    for (const auto& var : parseString(bhv_set_->getStateSpaceVars(), ','))
        if (!var.empty())
            c.add_state_variable(var);
    for (auto pair : parseString(bhv_set_->getUpdateVarSummary(), ','))
    {
        if (pair.empty())
            continue;
        auto& u = *c.add_update_variable();
        u.set_behavior(biteStringX(pair, '='));
        u.set_variable(pair);
    }
    for (const auto& var : registered_) c.add_subscription(var);

    const std::string serialized = c.SerializeAsString();
    if (serialized == configuration_)
        return;
    configuration_ = serialized;
    c.set_time(micros(curr_time_));
    output_.configuration = c;
}

// HelmIvP::OnNewMail ignores mail more than ok_skew from the helm's clock
bool Helm::Impl::skewed(const std::string& var, double msg_time)
{
    const double skew = std::fabs(curr_time_ - msg_time);
    if (ok_skew_ < 0 || skew <= ok_skew_ || var == "HELM_MAP_CLEAR")
        return false;
    const std::string msg = "Helm ignores " + var + " due to skew: " + doubleToString(skew, 2);
    add_behavior_event(BehaviorEvent::WARNING, "", msg);
    notify("IVPHELM_SKEW", msg + " curr_time: " + doubleToString(curr_time_, 3));
    return true;
}

void Helm::Impl::update_cpu_load()
{
    const std::clock_t cpu = std::clock();
    const auto wall = std::chrono::steady_clock::now();
    if (last_cpu_ != 0)
    {
        const double wall_seconds = std::chrono::duration<double>(wall - last_wall_).count();
        if (wall_seconds > 0)
            cpu_load_ =
                100.0 * static_cast<double>(cpu - last_cpu_) / CLOCKS_PER_SEC / wall_seconds;
    }
    last_cpu_ = cpu;
    last_wall_ = wall;
}

const Output& Helm::Impl::iterate(double time)
{
    output_ = Output();
    curr_time_ = time;
    update_cpu_load();

    info_buffer_.setCurrTime(time);
    ledger_.setCurrTimeUTC(time);
    if (!started_)
    {
        // pHelmIvP posts its "initialize" variables before it registers for
        // anything, so they replace whatever was posted before it started
        if (config_warnings_.empty())
            apply_pending(time);
        start(time);
    }

    // The previous iteration's posts (or the startup's) reach the helm first,
    // as they reached the MOOSDB first
    auto echo = std::move(posted_);
    posted_.clear();
    const double echo_time = posted_time_ == 0 ? time : posted_time_;
    posted_time_ = time;

    if (status_ != Status::MALCONFIG)
    {
        for (const auto& [var, value] : echo)
        {
            if (registered_.count(var))
                apply_variable(var, value, echo_time);
            else if (contact_manager_)
                contact_manager_->handle_mail(var, value.s);
        }
        apply_pending(time);
        info_buffer_.setValue("COMMS_POLICY", comms_policy_, time);
    }
    pending_.clear();

    update_status();
    if (status_ != Status::MALCONFIG)
        run_contact_manager();
    notify("IVPHELM_STATE", Status::State_Name(status_));
    fill_status();
    if (!enabled())
    {
        info_buffer_.clearDeltaVectors();
        return output_;
    }

    if (hold_for_nav_solution())
        return output_;

    if (!init_vars_done_ && time - start_time_ >= defer_window_)
        post_initial_variables(true);

    std::vector<std::string> keep_vnames = bhv_set_->getContactNames();
    ledger_.clearStaleNodes(keep_vnames, 2);
    ledger_.extrapolate();
    update_ledger_snap();

    if (!has_control_)
    {
        post_allstop("ManualOverride");
        fill_status();
        info_buffer_.clearDeltaVectors();
        return output_;
    }

    // no decision for the first second, to give inputs time to arrive
    if (time - start_time_ < 1)
        return output_;

    if (restart_post_pending_)
    {
        notify("IVPHELM_RESTARTED", "true");
        restart_post_pending_ = false;
    }

    update_plat_model();
    report_ = engine_->determineNextDecision(bhv_set_, time);
    iteration_ = report_.getIteration();

    if (refresh_pending_ && time - refresh_time_ > refresh_interval_)
    {
        outgoing_strings_.clear();
        outgoing_doubles_.clear();
        refresh_time_ = time;
        refresh_pending_ = false;
    }

    for (const auto& var : bhv_set_->getNewInfoVars()) register_variable(var);
    post_mode_messages();
    post_behavior_messages();
    post_life_events();
    post_decisions();

    if (bhv_set_->refreshMapUpdateVars())
        notify("IVPHELM_UPDATEVARS", bhv_set_->getUpdateVarSummary());
    update_configuration();

    fill_status();
    info_buffer_.clearDeltaVectors();
    return output_;
}

void Helm::Impl::apply_pending(double time)
{
    for (const auto& input : pending_)
    {
        if (const auto* nav = std::get_if<Navigation>(&input))
        {
            // a contact's time is when it was reported, which the ledger ages itself
            if (!nav->self() || nav->nav().time() == 0 ||
                !skewed("NAV", seconds(nav->nav().time(), time)))
                apply(*nav, time);
        }
        else if (const auto* var = std::get_if<Variable>(&input))
        {
            if (var->time() != 0 && skewed(var->key(), seconds(var->time(), time)))
                continue;
            Value value;
            value.is_string = var->value_case() == Variable::kSval;
            value.d = var->dval();
            value.s = var->sval();
            apply_variable(var->key(), value, seconds(var->time(), time));
        }
        else
            apply(std::get<Command>(input), time);
    }
    pending_.clear();
}

void Helm::Impl::apply(const Navigation& nav, double time)
{
    const auto& n = nav.nav();
    const double t = seconds(n.time(), time);

    if (!nav.self())
    {
        NodeRecord record = node_record(nav, cfg_.speed_mode());
        if (record.getName().empty())
            return;
        process_node_record(record);
        if (contact_manager_)
            contact_manager_->handle_node_record(record);
        info_buffer_.setValue("NODE_REPORT", record.getSpec(), t);
        return;
    }

    std::optional<std::pair<double, double>> xy, latlon;
    if (n.has_enu() && n.enu().has_e() && n.enu().has_n())
        xy = std::make_pair(n.enu().e(), n.enu().n());
    if (n.has_geodetic() && n.geodetic().has_latitude() && n.geodetic().has_longitude())
        latlon = std::make_pair(n.geodetic().latitude(), n.geodetic().longitude());
    if (!xy && latlon && geodesy_)
    {
        double x, y;
        geodesy_->to_local(latlon->first, latlon->second, x, y);
        xy = std::make_pair(x, y);
    }
    if (!latlon && xy && geodesy_)
    {
        double lat, lon;
        geodesy_->to_global(xy->first, xy->second, lat, lon);
        latlon = std::make_pair(lat, lon);
    }

    if (xy)
    {
        info_buffer_.setValue("NAV_X", xy->first, t);
        info_buffer_.setValue("NAV_Y", xy->second, t);
    }
    if (latlon)
    {
        info_buffer_.setValue("NAV_LAT", latlon->first, t);
        info_buffer_.setValue("NAV_LONG", latlon->second, t);
    }
    if (n.has_attitude() && n.attitude().has_heading())
        info_buffer_.setValue("NAV_HEADING", heading_degrees(n.attitude().heading()), t);
    if (auto speed = select_speed(n, cfg_.speed_mode()))
        info_buffer_.setValue("NAV_SPEED", *speed, t);
    if (n.has_geodetic() && n.geodetic().has_depth())
        info_buffer_.setValue("NAV_DEPTH", n.geodetic().depth(), t);
    if (n.has_altitude())
        info_buffer_.setValue("NAV_ALTITUDE", n.altitude(), t);

    NodeRecord local;
    local.setName(n.vehicle().name().empty() ? ownship_ : n.vehicle().name());
    local.setGroup(nav.group());
    local.setType(nav.type());
    local.setColor(nav.color());
    if (nav.has_length())
        local.setLength(nav.length());
    process_node_report_local(local);
}

void Helm::Impl::apply(const Command& command, double time)
{
    switch (command.command_case())
    {
        case Command::kManualOverride: set_manual_override(command.manual_override()); break;
        case Command::kRefresh:
            if (command.refresh())
                refresh_pending_ = true;
            break;
        case Command::kAbleFilter:
            apply_variable("BHV_ABLE_FILTER",
                           Value{true, 0, able_filter_string(command.able_filter())}, time);
            break;
        case Command::kRestart:
            restart(command.restart().type() == Command::Restart::REBUILD_AND_REINITIALIZE ? "type2"
                    : command.restart().type() == Command::Restart::REBUILD_BEHAVIORS      ? "type1"
                                                                                           : "");
            break;
        case Command::kCommsPolicy:
            comms_policy_ = tolower(Command::CommsPolicy_Name(command.comms_policy()));
            break;
        default: break;
    }
}

// HelmIvP::OnNewMail for one message
void Helm::Impl::apply_variable(const std::string& var, const Value& value, double msg_time)
{
    // pHelmIvP has set the helm's own "initialize" values already, but still
    // acts on them (e.g. initialize MOOS_MANUAL_OVERRIDE = false)
    auto store = [&]()
    {
        if (value.init)
            return;
        if (value.is_string)
            info_buffer_.setValue(var, value.s, msg_time);
        else
            info_buffer_.setValue(var, value.d, msg_time);
    };

    const std::string sval = value.s;
    if (contact_manager_)
        contact_manager_->handle_mail(var, sval);

    if (var == "MOOS_MANUAL_OVERRIDE" || var == "MOOS_MANUAL_OVERIDE")
    {
        if (toupper(sval) == "FALSE")
            set_manual_override(false);
        else if (toupper(sval) == "TRUE")
            set_manual_override(true);
        store();
    }
    else if (var == "RESTART_HELM")
        restart(sval);
    // Status is always complete, so there's nothing to rejournal
    else if (var == "IVPHELM_STATE" || var == "DB_CLIENTS" || var == "IVPHELM_REJOURNAL") {}
    else if (var == "BHV_ABLE_FILTER")
        engine_->addAbleFilterMsg(sval);
    else if (var == "NODE_REPORT_LOCAL")
        process_node_report_local(string2NodeRecord(sval));
    else if (var == "NODE_REPORT" || var == "AIS_REPORT" || var == "AIS_REPORT_LOCAL")
    {
        std::string whynot;
        const std::string vname = ledger_.processNodeReport(sval, whynot);
        if (vname.empty())
            add_behavior_event(BehaviorEvent::WARNING, "", "Bad node report: " + whynot);
        else
        {
            info_buffer_.setValue(toupper(vname) + "_NAV_GROUP", ledger_.getGroup(vname));
            info_buffer_.setValue(toupper(vname) + "_NAV_TYPE", ledger_.getType(vname));
        }
        store();
    }
    else if (var == "HELM_MAP_CLEAR")
        refresh_pending_ = true;
    else
        store();
}

void Helm::Impl::set_manual_override(bool on)
{
    if (!on)
        has_control_ = true;
    else if (allow_park_)
        has_control_ = false;
    update_status();
}

bool Helm::Impl::process_node_record(NodeRecord record)
{
    std::string whynot;
    const std::string vname = ledger_.processNodeRecord(record, whynot);
    if (vname.empty())
    {
        add_behavior_event(BehaviorEvent::WARNING, "", "Bad node report: " + whynot);
        return false;
    }
    info_buffer_.setValue(toupper(vname) + "_NAV_GROUP", ledger_.getGroup(vname));
    info_buffer_.setValue(toupper(vname) + "_NAV_TYPE", ledger_.getType(vname));
    return true;
}

void Helm::Impl::process_node_report_local(const NodeRecord& record)
{
    const std::string vname = toupper(record.getName());
    if (vname.empty())
        return;
    if (!record.getGroup().empty())
        info_buffer_.setValue(vname + "_NAV_GROUP", record.getGroup());
    if (!record.getType().empty())
        info_buffer_.setValue(vname + "_NAV_TYPE", record.getType());
    if (!record.getColor().empty())
        info_buffer_.setValue(vname + "_NAV_COLOR", record.getColor());
    if (record.getLength() != 0)
        info_buffer_.setValue(vname + "_NAV_LENGTH", record.getLength());
}

void Helm::Impl::register_variable(const std::string& var)
{
    if (registered_.insert(var).second)
        notify("IVPHELM_REGISTER", var);
}

void Helm::Impl::register_variables()
{
    for (const char* var :
         {"MOOS_MANUAL_OVERIDE", "MOOS_MANUAL_OVERRIDE", "RESTART_HELM", "IVPHELM_REJOURNAL",
          "BHV_ABLE_FILTER", "NAV_X", "NAV_Y", "NAV_SPEED", "NAV_HEADING", "NAV_DEPTH",
          "HELM_MAP_CLEAR", "AIS_REPORT", "NODE_REPORT", "AIS_REPORT_LOCAL", "NODE_REPORT_LOCAL",
          "IVPHELM_STATE"})
        register_variable(var);
    for (const auto& var : bhv_set_->getInfoVars()) register_variable(var);
    for (const auto& var : bhv_set_->getSpecUpdateVars()) register_variable(var);
}

void Helm::Impl::update_status()
{
    if (status_ == Status::PARK && has_control_)
        status_ = Status::DRIVE;
    else if (status_ == Status::DRIVE && !has_control_)
        status_ = Status::PARK;
}

bool Helm::Impl::hold_for_nav_solution()
{
    if (nav_started_)
        return false;

    std::vector<std::string> vars{"NAV_X", "NAV_Y", "NAV_HEADING", "NAV_SPEED"};
    if (domain_.hasDomain("depth"))
        vars.push_back("NAV_DEPTH");
    std::string missing;
    for (const auto& var : vars)
        if (!info_buffer_.isKnown(var))
            missing += (missing.empty() ? "" : ",") + var;

    if (missing.empty())
    {
        nav_started_ = true;
        return false;
    }
    if (curr_time_ - start_time_ > nav_grace_)
        add_behavior_event(BehaviorEvent::WARNING, "", "Waiting for helm nav solution: " + missing);
    return true;
}

// pHelmIvP's handleInitialVarsPhase1 (deferred = false) and Phase2 (deferred = true)
void Helm::Impl::post_initial_variables(bool deferred)
{
    if (deferred)
        init_vars_done_ = true;
    for (const VarDataPair& pair : bhv_set_->getInitialVariables())
    {
        const std::string var = stripBlankEnds(pair.get_var());
        const std::string key = tolower(pair.get_key());
        if (strContainsWhite(var))
            continue;
        if (!deferred && key == "defer")
            register_variable(var);
        if ((!deferred && key == "post") ||
            (deferred && key == "defer" && !info_buffer_.isKnown(var)))
        {
            Value value = make_value(pair);
            value.s = stripBlankEnds(value.s);
            value.is_string = !value.s.empty();
            if (value.is_string)
                info_buffer_.setValue(var, value.s);
            else
                info_buffer_.setValue(var, value.d);
            add_posting(Posting::INITIALIZE, "", var, value);
            value.init = true;
            notify(var, value);
        }
    }
}

void Helm::Impl::post_helm_start_messages()
{
    if (helm_start_posted_)
        return;
    helm_start_posted_ = true;
    for (const VarDataPair& pair : bhv_set_->getHelmStartMessages())
    {
        const std::string var = stripBlankEnds(pair.get_var());
        if (strContainsWhite(var))
            continue;
        Value value = make_value(pair);
        value.s = stripBlankEnds(value.s);
        value.is_string = !value.s.empty();
        if (value.is_string)
            info_buffer_.setValue(var, value.s);
        else
            info_buffer_.setValue(var, value.d);
        add_posting(Posting::HELM_START, "", var, value);
        notify(var, value);
    }
}

void Helm::Impl::update_ledger_snap()
{
    ledger_snap_.clear();
    for (const auto& v : ledger_.getVNames())
    {
        ledger_snap_.setX(v, ledger_.getX(v));
        ledger_snap_.setY(v, ledger_.getY(v));
        ledger_snap_.setHdg(v, ledger_.getHeading(v));
        ledger_snap_.setSpd(v, ledger_.getSpeed(v));
        ledger_snap_.setDep(v, ledger_.getDepth(v));
        ledger_snap_.setLat(v, ledger_.getLat(v));
        ledger_snap_.setLon(v, ledger_.getLon(v));
        ledger_snap_.setUTC(v, ledger_.getUTC(v));
        ledger_snap_.setUTCAge(v, ledger_.getUTCAge(v));
        ledger_snap_.setUTCReceived(v, ledger_.getUTCReceived(v));
        ledger_snap_.setUTCAgeReceived(v, ledger_.getUTCAgeReceived(v));
        ledger_snap_.setGroup(v, ledger_.getGroup(v));
        ledger_snap_.setType(v, ledger_.getType(v));
        ledger_snap_.setVSource(v, ledger_.getVSource(v));
    }
    ledger_snap_.setCurrTimeUTC(curr_time_);
}

void Helm::Impl::run_contact_manager()
{
    if (!contact_manager_)
        return;
    bool ok1, ok2, ok3, ok4;
    const double osx = info_buffer_.dQuery("NAV_X", ok1);
    const double osy = info_buffer_.dQuery("NAV_Y", ok2);
    const double osh = info_buffer_.dQuery("NAV_HEADING", ok3);
    const double osv = info_buffer_.dQuery("NAV_SPEED", ok4);
    contact_manager_->set_ownship(ok1 ? osx : 0, ok2 ? osy : 0, ok3 ? osh : 0, ok4 ? osv : 0);

    for (const auto& post : contact_manager_->iterate(curr_time_, status_ == Status::DRIVE))
    {
        const Value value{post.is_string, post.d, post.s};
        add_posting(Posting::CONTACT_MANAGER, "", post.var, value);
        notify(post.var, value);
    }
}

void Helm::Impl::update_plat_model()
{
    bool ok1, ok2, ok3, ok4;
    const double osx = info_buffer_.dQuery("NAV_X", ok1);
    const double osy = info_buffer_.dQuery("NAV_Y", ok2);
    const double osh = info_buffer_.dQuery("NAV_HEADING", ok3);
    const double osv = info_buffer_.dQuery("NAV_SPEED", ok4);
    if (!ok1 || !ok2 || !ok3 || !ok4)
        return;

    pmgen_.setCurrTime(info_buffer_.getCurrTime());
    engine_->setPlatModel(pmgen_.generate(osx, osy, osh, osv));
    const double turn_rate = pmgen_.getTurnRate();
    output_.status.set_turn_rate(turn_rate);
    notify("TURN_RATE", turn_rate);
}

void Helm::Impl::post_mode_messages()
{
    for (const VarDataPair& pair : bhv_set_->getModeVarDataPairs())
    {
        const Value value = make_value(pair);
        const bool changed = value.is_string ? detect_change(pair.get_key(), value.s)
                                             : detect_change(pair.get_key(), value.d);
        if (changed)
        {
            add_posting(Posting::MODE, "", pair.get_var(), value);
            notify(pair.get_var(), value);
        }
    }
}

void Helm::Impl::post_behavior_messages()
{
    for (const auto& warning : bhv_set_->getWarnings())
        add_behavior_event(BehaviorEvent::WARNING, "", warning);
    bhv_set_->clearWarnings();

    notify("IVPHELM_ITER", iteration_);
    for (unsigned i = 0; i < bhv_set_->size(); ++i)
    {
        const std::string bhv = bhv_set_->getDescriptor(i);
        for (const VarDataPair& msg : bhv_set_->getMessages(i))
        {
            const std::string var = msg.get_var();
            const std::string key = msg.get_key();
            const Value value = make_value(msg);

            if (var == "BHV_IPF")
            {
                auto& of = output_.objective_functions.emplace_back();
                of.set_time(micros(curr_time_));
                of.set_behavior(bhv);
                of.set_encoding(value.s);
                of.set_iteration(iteration_);
                continue;
            }

            // a string with an empty value is compared as a number, as pHelmIvP does
            const bool changed =
                value.s.empty() ? detect_change(key, value.d) : detect_change(key, value.s);

            if (value.is_string)
                info_buffer_.setValue(var, value.s);
            else
                info_buffer_.setValue(var, value.d);

            if (!changed && key != "repeatable")
                continue;

            if (auto kind = behavior_event_kind(var, key))
                add_behavior_event(*kind, bhv,
                                   value.is_string ? value.s : doubleToStringX(value.d));
            else if (auto vis = value.is_string ? visualization(var, value.s) : std::nullopt)
            {
                vis->set_time(micros(curr_time_));
                vis->set_behavior(bhv);
                vis->set_iteration(iteration_);
                output_.visualizations.push_back(*vis);
            }
            else if (auto msg = var == "NODE_MESSAGE_LOCAL" && value.is_string
                                    ? offboard_message(value.s)
                                    : std::nullopt)
            {
                msg->set_time(micros(curr_time_));
                output_.offboard_messages.push_back(*msg);
            }
            else
                add_posting(Posting::BEHAVIOR, bhv, var, value);
            notify(var, value);
        }
    }

    if (bhv_set_->updateStateSpaceVars())
        notify("IVPHELM_STATEVARS", bhv_set_->getStateSpaceVars());
    bhv_set_->removeCompletedBehaviors();
}

void Helm::Impl::post_life_events()
{
    for (const LifeEvent& event : bhv_set_->getLifeEvents())
    {
        auto& out = output_.life_events.emplace_back();
        out.set_time(micros(curr_time_));
        const std::string type = event.getEventType();
        out.set_type(type == "spawn"   ? ivp::interface::LifeEvent::SPAWN
                     : type == "death" ? ivp::interface::LifeEvent::DEATH
                     : type == "abort" ? ivp::interface::LifeEvent::ABORT
                                       : ivp::interface::LifeEvent::TYPE_UNKNOWN);
        out.set_behavior(event.getBehaviorName());
        out.set_behavior_type(event.getBehaviorType());
        out.set_spawn_string(event.getSpawnString());
        out.set_post_mortem(event.getPostMortem());
        out.set_iteration(iteration_);
        notify("IVPHELM_LIFE_EVENT",
               event.getSpec(curr_time_ - start_time_) + ",iter=" + uintToString(iteration_));
    }
    bhv_set_->clearLifeEvents();
}

void Helm::Impl::post_decisions()
{
    std::string allstop_msg = "clear";
    if (report_.getHalted())
        allstop_msg = "BehaviorError";
    else if (report_.getOFNUM() == 0)
        allstop_msg = "NothingToDo";
    else if (cfg_.goals_mandatory() && !report_.getActiveGoal())
        allstop_msg = "NoGoalBehavior";

    missing_decision_vars_.clear();
    if (allstop_msg == "clear")
    {
        for (unsigned i = 0; i < domain_.size(); ++i)
        {
            const std::string var = domain_.getVarName(i);
            if (!report_.hasDecision(var) && !optional_vars_.count(var))
                missing_decision_vars_.push_back(var);
        }
        if (!missing_decision_vars_.empty())
        {
            allstop_msg = "MissingDecVars:";
            for (std::size_t i = 0; i < missing_decision_vars_.size(); ++i)
                allstop_msg += (i ? "," : "") + missing_decision_vars_[i];
            add_behavior_event(BehaviorEvent::ERROR, "", allstop_msg);
            notify("BHV_ERROR", allstop_msg);
        }
    }

    post_allstop(allstop_msg);
    if (allstop_msg != "clear")
        return;

    ivp::interface::Solution solution;
    solution.set_allstop(ivp::interface::CLEAR);
    solution.set_iteration(iteration_);
    auto& desired = *solution.mutable_desired();
    desired.set_time(micros(curr_time_));
    for (unsigned i = 0; i < domain_.size(); ++i)
    {
        const std::string var = domain_.getVarName(i);
        if (!report_.hasDecision(var))
            continue;
        const double value = report_.getDecision(var);
        set_desired(desired, var, value);
        notify(desired_variable(var), value);
    }
    output_.solution = solution;
}

// HelmIvP::postAllStop, but the zero solution goes out every iteration of an allstop:
// a frontseat holds its last command, where pMarinePID stops on stale DESIRED_*
void Helm::Impl::post_allstop(const std::string& msg)
{
    if (!enabled())
        return;

    if (msg != allstop_msg_)
    {
        allstop_msg_ = msg;
        allstop_reason_ = msg;
        no_decisions_ =
            (msg == "NothingToDo" || msg == "NoGoalBehavior" || strBegins(msg, "MissingDecVars"))
                ? no_decisions_ + 1
                : 0;

        notify("IVPHELM_ALLSTOP", msg);
        if (msg == "clear")
            return;

        // one iteration's grace to let the helm move between modes
        if (no_decisions_ == 1)
            return;

        // other allstops keep control, and the helm decides again once they clear
        if (allow_park_ && park_on_allstop_ &&
            (msg == "BehaviorError" || msg == "DisabledByStandbyHelm"))
            has_control_ = false;

        for (unsigned i = 0; i < domain_.size(); ++i)
            notify(desired_variable(domain_.getVarName(i)), 0.0);
    }
    else if (msg == "clear")
        return;

    ivp::interface::Solution solution;
    solution.set_allstop(allstop_enum(msg));
    solution.set_iteration(iteration_);
    auto& desired = *solution.mutable_desired();
    desired.set_time(micros(curr_time_));
    for (unsigned i = 0; i < domain_.size(); ++i) set_desired(desired, domain_.getVarName(i), 0);
    output_.solution = solution;
}

void Helm::Impl::fill_status()
{
    Status& s = output_.status;
    s.set_time(micros(curr_time_));
    s.set_state(status_);
    s.set_allstop(allstop_enum(allstop_reason_));
    s.set_iteration(iteration_);
    s.clear_missing_decision_variables();
    for (const auto& var : missing_decision_vars_) s.add_missing_decision_variables(var);
    s.set_halt_message(report_.getHalted() ? report_.getHaltMsg() : "");

    s.clear_mode();
    s.clear_behavior();
    s.clear_contact();
    if (!bhv_set_)
        return;

    for (const VarDataPair& pair : bhv_set_->getModeVarDataPairs())
    {
        auto& mode = *s.add_mode();
        mode.set_variable(pair.get_var());
        mode.set_value(pair.is_string() ? pair.get_sdata() : doubleToStringX(pair.get_ddata()));
    }

    // HelmReport's full lists: "name$time$updates" entries separated by ':', and
    // for active behaviors "name$time$pwt$pieces$cpu$updates$ipfs$mode$submode"
    std::map<std::string, Status::Behavior> details;
    auto add_states = [&](const std::string& list, Status::Behavior::RunState state)
    {
        for (const auto& entry : parseString(list, ':'))
        {
            const auto fields = parseString(entry, '$');
            if (fields.empty() || fields[0].empty())
                continue;
            Status::Behavior& b = details[fields[0]];
            b.Clear();
            b.set_run_state(state);
            if (state == Status::Behavior::ACTIVE && fields.size() >= 6)
            {
                b.set_priority_weight(atof(fields[2].c_str()));
                b.set_pieces(static_cast<unsigned>(atoi(fields[3].c_str())));
                b.set_create_cpu(atof(fields[4].c_str()));
                b.set_update_summary(fields[5]);
            }
            else if (fields.size() >= 3)
                b.set_update_summary(fields[2]);
        }
    };
    // first, since a respawned behavior keeps the name of one that completed
    add_states(report_.getCompletedBehaviors(true), Status::Behavior::COMPLETED);
    add_states(report_.getIdleBehaviors(true), Status::Behavior::IDLE);
    add_states(report_.getRunningBehaviors(true), Status::Behavior::RUNNING);
    add_states(report_.getActiveBehaviors(true), Status::Behavior::ACTIVE);
    add_states(report_.getDisabledBehaviors(true), Status::Behavior::DISABLED);

    for (unsigned i = 0; i < bhv_set_->size(); ++i)
    {
        auto& b = *s.add_behavior();
        const std::string name = bhv_set_->getDescriptor(i);
        auto it = details.find(name);
        if (it != details.end())
            b = it->second;
        b.set_name(name);
        b.set_type(bhv_set_->getBehavior(i)->getBehaviorType());
        b.set_state_time_entered(micros(bhv_set_->getStateTimeEntered(i)));
        b.set_mode(bhv_set_->getBehaviorMode(i));
        b.set_submode(bhv_set_->getBehaviorSubMode(i));
    }
    s.set_completed_count(report_.getCompletedCnt());
    s.set_warning_count(warning_count_);
    s.set_cpu_load(cpu_load_);

    s.clear_update_result();
    for (const auto& result : report_.getUpdateResults())
    {
        auto& u = *s.add_update_result();
        u.set_behavior(tokStringParse(result, "bhv", ',', '='));
        u.set_variable(tokStringParse(result, "var", ',', '='));
        const std::string r = tokStringParse(result, "result", ',', '=');
        u.set_result(r == "success"         ? Status::UpdateResult::SUCCESS
                     : r == "duplicate"     ? Status::UpdateResult::DUPLICATE
                     : r == "name_mismatch" ? Status::UpdateResult::NAME_MISMATCH
                     : r == "param_error"   ? Status::UpdateResult::PARAM_ERROR
                                            : Status::UpdateResult::RESULT_UNKNOWN);
    }
    s.set_behavior_count(bhv_set_->size());
    s.set_behavior_count_ever(bhv_set_->getTCount());
    s.set_completed_pending(bhv_set_->getCompletedPending());

    for (const auto& vname : ledger_.getVNames()) s.add_contact(vname);

    s.set_objective_function_count(report_.getOFNUM());
    s.set_total_pieces_formed(report_.getTotalPcsFormed());
    s.set_total_pieces_cached(report_.getTotalPcsCached());
    s.set_create_cpu(report_.getCreateTime());
    s.set_solve_cpu(report_.getSolveTime());
    s.set_loop_cpu(report_.getLoopTime());
    s.set_max_create_cpu(report_.getMaxCreateTime());
    s.set_max_solve_cpu(report_.getMaxSolveTime());
    s.set_max_loop_cpu(report_.getMaxLoopTime());
}

bool Helm::Impl::detect_change(const std::string& key, const std::string& value)
{
    if (key.empty())
        return true;
    auto [it, inserted] = outgoing_strings_.emplace(key, value);
    if (inserted)
        return true;
    if (it->second == value)
        return false;
    it->second = value;
    return true;
}

bool Helm::Impl::detect_change(const std::string& key, double value)
{
    if (key.empty())
        return true;
    auto [it, inserted] = outgoing_doubles_.emplace(key, value);
    if (inserted)
        return true;
    if (it->second == value)
        return false;
    it->second = value;
    return true;
}

void Helm::Impl::notify(const std::string& var, const Value& value)
{
    posted_.emplace_back(var, value);
}

Posting& Helm::Impl::add_posting(Posting::Origin origin, const std::string& behavior,
                                 const std::string& var, const Value& value)
{
    Posting& posting = output_.postings.emplace_back();
    posting.set_time(micros(curr_time_));
    posting.set_origin(origin);
    posting.set_behavior(behavior);
    posting.set_key(var);
    set_value(posting, value);
    posting.set_iteration(iteration_);
    return posting;
}

void Helm::Impl::add_behavior_event(BehaviorEvent::Kind kind, const std::string& behavior,
                                    const std::string& text)
{
    if (kind == BehaviorEvent::WARNING)
        ++warning_count_;
    BehaviorEvent& event = output_.behavior_events.emplace_back();
    event.set_time(micros(curr_time_));
    event.set_behavior(behavior);
    event.set_kind(kind);
    event.set_text(text);
    event.set_iteration(iteration_);
}

Helm::Helm(const HelmConfig& cfg, std::optional<Geodesy> geodesy)
    : impl_(std::make_unique<Impl>(cfg, std::move(geodesy)))
{
}

Helm::~Helm() = default;

void Helm::handle(const Navigation& navigation) { impl_->queue(navigation); }
void Helm::handle(const Variable& variable) { impl_->queue(variable); }
void Helm::handle(const Command& command) { impl_->queue(command); }

const Output& Helm::iterate(double time) { return impl_->iterate(time); }

Status::State Helm::state() const { return impl_->state(); }

const std::vector<std::string>& Helm::config_warnings() const { return impl_->config_warnings(); }

} // namespace helm
} // namespace ivpico
