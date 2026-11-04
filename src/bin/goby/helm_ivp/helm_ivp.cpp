#include <goby/middleware/marshalling/protobuf.h>

#include <cmath>

#include <goby/middleware/frontseat/groups.h>
#include <goby/middleware/protobuf/frontseat.pb.h>
#include <goby/middleware/protobuf/frontseat_data.pb.h>
#include <goby/time/system_clock.h>
#include <goby/zeromq/application/single_thread.h>

#include "groups.h"
#include "helm/helm.h"
#include "helm_ivp_config.pb.h"

using goby::glog;
namespace frontseat = goby::middleware::frontseat;

namespace ivpico
{
namespace goby
{
class HelmIvP : public ::goby::zeromq::SingleThreadApplication<protobuf::HelmIvPConfig>
{
  public:
    HelmIvP();

  private:
    void loop() override;
    void health(::goby::middleware::protobuf::ThreadHealth& health) override;

    ivp::interface::HelmConfig helm_config();
    std::optional<helm::Geodesy> make_geodesy();
    void handle_node_status(const ::goby::middleware::frontseat::protobuf::NodeStatus& status);
    void publish(const helm::Output& out);
    void publish_frontseat(const helm::Output& out);
    void log(const ivp::interface::BehaviorEvent& event);

    helm::Helm helm_;
    ivp::interface::Status::State state_{ivp::interface::Status::STATE_UNKNOWN};
    std::string halt_message_;
};
} // namespace goby
} // namespace ivpico

namespace
{
constexpr double degrees(double radians) { return radians * 180.0 / M_PI; }
constexpr double radians(double degrees) { return degrees * M_PI / 180.0; }
} // namespace

int main(int argc, char* argv[]) { return goby::run<ivpico::goby::HelmIvP>(argc, argv); }

ivpico::goby::HelmIvP::HelmIvP()
    // cfg() isn't available until the base classes are constructed
    : ::goby::zeromq::SingleThreadApplication<protobuf::HelmIvPConfig>(1 * boost::units::si::hertz),
      helm_(helm_config(), make_geodesy())
{
    set_loop_frequency_hertz(cfg().loop_freq());

    for (const auto& warning : helm_.config_warnings())
        glog.is_warn() && glog << "Helm configuration: " << warning << std::endl;

    interprocess().subscribe<groups::navigation>([this](const ivp::interface::Navigation& nav)
                                                 { helm_.handle(nav); });
    interprocess().subscribe<groups::variable>([this](const ivp::interface::Variable& variable)
                                               { helm_.handle(variable); });
    interprocess().subscribe<groups::command>([this](const ivp::interface::Command& command)
                                              { helm_.handle(command); });

    if (cfg().frontseat().enable() && cfg().frontseat().use_node_status())
        interprocess().subscribe<frontseat::groups::node_status>(
            [this](const ::goby::middleware::frontseat::protobuf::NodeStatus& status)
            { handle_node_status(status); });
}

ivp::interface::HelmConfig ivpico::goby::HelmIvP::helm_config()
{
    auto helm = cfg().helm();
    if (helm.ownship().empty())
        helm.set_ownship(cfg().interprocess().platform());
    return helm;
}

std::optional<ivpico::helm::Geodesy> ivpico::goby::HelmIvP::make_geodesy()
{
    // goby's datum (app.geodesy or a datum_update) must be set at startup
    if (!has_geodesy())
        return std::nullopt;

    helm::Geodesy geodesy;
    geodesy.to_local = [this](double lat, double lon, double& x, double& y)
    {
        auto xy = this->geodesy().convert(
            {lat * boost::units::degree::degrees, lon * boost::units::degree::degrees});
        x = xy.x.value();
        y = xy.y.value();
    };
    geodesy.to_global = [this](double x, double y, double& lat, double& lon)
    {
        auto latlon =
            this->geodesy().convert({x * boost::units::si::meters, y * boost::units::si::meters});
        lat = latlon.lat.value();
        lon = latlon.lon.value();
    };
    return geodesy;
}

void ivpico::goby::HelmIvP::handle_node_status(
    const ::goby::middleware::frontseat::protobuf::NodeStatus& status)
{
    ivp::interface::Navigation nav;
    nav.set_self(true);
    auto& n = *nav.mutable_nav();
    n.set_time(static_cast<int64_t>(std::llround(status.time() * 1e6)));
    if (status.has_name())
        n.mutable_vehicle()->set_name(status.name());

    auto& geodetic = *n.mutable_geodetic();
    geodetic.set_latitude(status.global_fix().lat());
    geodetic.set_longitude(status.global_fix().lon());
    geodetic.set_depth(status.global_fix().depth());
    if (status.global_fix().has_altitude() && !std::isnan(status.global_fix().altitude()))
        n.set_altitude(status.global_fix().altitude());
    if (status.has_local_fix())
    {
        n.mutable_enu()->set_e(status.local_fix().x());
        n.mutable_enu()->set_n(status.local_fix().y());
    }

    auto& attitude = *n.mutable_attitude();
    if (status.pose().has_heading())
        attitude.set_heading(radians(status.pose().heading()));
    if (status.pose().has_pitch())
        attitude.set_pitch(radians(status.pose().pitch()));
    if (status.pose().has_roll())
        attitude.set_roll(radians(status.pose().roll()));

    auto& over_ground = *n.add_speed();
    over_ground.set_value(status.speed().over_ground());
    over_ground.set_mode(openocean::SPEED_MODE_OVER_GROUND);
    if (status.speed().has_over_water())
    {
        auto& over_water = *n.add_speed();
        over_water.set_value(status.speed().over_water());
        over_water.set_mode(openocean::SPEED_MODE_OVER_WATER);
    }

    helm_.handle(nav);
}

void ivpico::goby::HelmIvP::loop()
{
    const auto& out = helm_.iterate(::goby::time::SystemClock::now<::goby::time::SITime>().value());
    state_ = out.status.state();
    halt_message_ = out.status.halt_message();
    publish(out);
    if (cfg().frontseat().enable())
        publish_frontseat(out);
}

void ivpico::goby::HelmIvP::publish(const helm::Output& out)
{
    if (out.solution)
        interprocess().publish<groups::solution>(*out.solution);
    interprocess().publish<groups::status>(out.status);
    for (const auto& p : out.postings) interprocess().publish<groups::posting>(p);
    for (const auto& e : out.behavior_events)
    {
        log(e);
        interprocess().publish<groups::behavior_event>(e);
    }
    for (const auto& v : out.visualizations) interprocess().publish<groups::visualization>(v);
    for (const auto& m : out.offboard_messages) interprocess().publish<groups::offboard_message>(m);
    for (const auto& e : out.life_events) interprocess().publish<groups::life_event>(e);
    for (const auto& f : out.objective_functions)
        interprocess().publish<groups::objective_function>(f);
    if (out.configuration)
        interprocess().publish<groups::configuration>(*out.configuration);
}

void ivpico::goby::HelmIvP::publish_frontseat(const helm::Output& out)
{
    using ::goby::middleware::frontseat::protobuf::HelmStateReport;

    if (cfg().frontseat().publish_desired_course() && out.solution)
    {
        const auto& desired = out.solution->desired();
        ::goby::middleware::frontseat::protobuf::DesiredCourse course;
        course.set_time(static_cast<double>(desired.time()) * 1e-6);
        if (desired.has_attitude() && desired.attitude().has_heading())
            course.set_heading(degrees(desired.attitude().heading()));
        if (desired.has_speed() && desired.speed().has_value())
            course.set_speed(desired.speed().value());
        if (desired.has_depth())
            course.set_depth(desired.depth());
        if (desired.has_altitude())
            course.set_altitude(desired.altitude());
        if (desired.has_attitude() && desired.attitude().has_pitch())
            course.set_pitch(degrees(desired.attitude().pitch()));
        if (desired.has_attitude() && desired.attitude().has_roll())
            course.set_roll(degrees(desired.attitude().roll()));
        interprocess().publish<frontseat::groups::desired_course>(course);
    }

    if (cfg().frontseat().publish_helm_state())
    {
        HelmStateReport report;
        switch (out.status.state())
        {
            case ivp::interface::Status::DRIVE:
                report.set_state(::goby::middleware::frontseat::protobuf::HELM_DRIVE);
                break;
            case ivp::interface::Status::PARK:
                report.set_state(::goby::middleware::frontseat::protobuf::HELM_PARK);
                break;
            default:
                report.set_state(::goby::middleware::frontseat::protobuf::HELM_NOT_RUNNING);
                break;
        }
        interprocess().publish<frontseat::groups::helm_state>(report);
    }
}

void ivpico::goby::HelmIvP::log(const ivp::interface::BehaviorEvent& event)
{
    using ivp::interface::BehaviorEvent;
    const std::string who = event.behavior().empty() ? "helm" : event.behavior();
    switch (event.kind())
    {
        case BehaviorEvent::WARNING:
        case BehaviorEvent::ERROR:
        case BehaviorEvent::CONFIG_WARNING:
        case BehaviorEvent::BAD_CONFIG:
            glog.is_warn() && glog << who << ": " << event.text() << std::endl;
            break;
        case BehaviorEvent::EVENT:
        case BehaviorEvent::WARNING_RETRACTED:
            glog.is_verbose() && glog << who << ": " << event.text() << std::endl;
            break;
        default: glog.is_debug1() && glog << who << ": " << event.text() << std::endl; break;
    }
}

void ivpico::goby::HelmIvP::health(::goby::middleware::protobuf::ThreadHealth& health)
{
    SingleThreadApplication::health(health);
    if (state_ == ivp::interface::Status::MALCONFIG)
    {
        health.set_state(::goby::middleware::protobuf::HEALTH__FAILED);
        std::string message = "MALCONFIG";
        for (const auto& warning : helm_.config_warnings()) message += "; " + warning;
        health.set_error_message(message);
    }
    else if (!halt_message_.empty())
    {
        health.set_state(::goby::middleware::protobuf::HEALTH__DEGRADED);
        health.set_error_message(halt_message_);
    }
}
