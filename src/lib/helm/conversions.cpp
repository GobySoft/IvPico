#include "conversions.h"

#include <cmath>

#include "MBUtils.h"

namespace ivpico
{
namespace helm
{
namespace
{
constexpr double degrees(double radians) { return radians * 180.0 / M_PI; }
constexpr double radians(double degrees) { return degrees * M_PI / 180.0; }
} // namespace

double heading_degrees(double radians)
{
    const double h = std::fmod(degrees(radians), 360.0);
    return h < 0 ? h + 360.0 : h;
}

std::optional<double> select_speed(const openocean::Navigation& nav, openocean::SpeedMode mode)
{
    std::optional<double> first;
    for (const auto& speed : nav.speed())
    {
        if (!speed.has_value())
            continue;
        if (speed.mode() == mode)
            return speed.value();
        if (!first)
            first = speed.value();
    }
    return first;
}

NodeRecord node_record(const ivp::interface::Navigation& nav, openocean::SpeedMode speed_mode)
{
    const auto& n = nav.nav();
    NodeRecord record;
    record.setName(n.vehicle().name());
    record.setTimeStamp(static_cast<double>(n.time()) * 1e-6);
    if (n.has_geodetic() && n.geodetic().has_latitude() && n.geodetic().has_longitude())
    {
        record.setLat(n.geodetic().latitude());
        record.setLon(n.geodetic().longitude());
    }
    if (n.has_enu() && n.enu().has_e() && n.enu().has_n())
    {
        record.setX(n.enu().e());
        record.setY(n.enu().n());
    }
    if (n.has_attitude() && n.attitude().has_heading())
        record.setHeading(heading_degrees(n.attitude().heading()));
    if (auto speed = select_speed(n, speed_mode))
        record.setSpeed(*speed);
    if (n.has_geodetic() && n.geodetic().has_depth())
        record.setDepth(n.geodetic().depth());
    if (n.has_altitude())
        record.setAltitude(n.altitude());
    if (n.has_course_over_ground())
        record.setCourseOG(heading_degrees(n.course_over_ground()));

    record.setType(nav.type());
    record.setGroup(nav.group());
    record.setColor(nav.color());
    record.setVSource(nav.vsource());
    if (nav.has_length())
        record.setLength(nav.length());
    if (nav.has_beam())
        record.setBeam(nav.beam());
    return record;
}

std::string able_filter_string(const ivp::interface::Command::AbleFilter& filter)
{
    using AbleFilter = ivp::interface::Command::AbleFilter;
    std::string s = "action=" + tolower(AbleFilter::Action_Name(filter.action()));
    if (!filter.contact().empty())
        s += ",contact=" + filter.contact();
    if (!filter.behavior_type().empty())
        s += ",bhv_type=" + filter.behavior_type();
    if (!filter.vsource().empty())
        s += ",vsource=" + filter.vsource();
    if (!filter.obstacle_id().empty())
        s += ",obstacle_id=" + filter.obstacle_id();
    return s;
}

void set_desired(openocean::ControlSetpoint& desired, const std::string& domain_var, double value)
{
    const std::string var = tolower(domain_var);
    if (var == "course" || var == "heading")
        desired.mutable_attitude()->set_heading(radians(value));
    else if (var == "speed")
        desired.mutable_speed()->set_value(value);
    else if (var == "depth")
        desired.set_depth(value);
    else if (var == "altitude")
        desired.set_altitude(value);
    else if (var == "pitch")
        desired.mutable_attitude()->set_pitch(radians(value));
    else if (var == "roll")
        desired.mutable_attitude()->set_roll(radians(value));
    else
    {
        auto& custom = *desired.add_custom();
        custom.set_name("DESIRED_" + toupper(domain_var));
        custom.set_value(value);
    }
}

} // namespace helm
} // namespace ivpico
