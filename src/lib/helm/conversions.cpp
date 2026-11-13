// Copyright 2026:
//   GobySoft, LLC
//
// This file is part of IvPico.
//
// IvPico is free software: you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the Free Software
// Foundation, either version 2.1 of the License, or (at your option) any later
// version.
//
// IvPico is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with IvPico. If not, see <https://www.gnu.org/licenses/>.

#include "conversions.h"

#include <cmath>

#include "MBUtils.h"
#include "NodeMessage.h"
#include "NodeMessageUtils.h"
#include "XYFormatUtilsPoint.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsSegl.h"
#include "XYFormatUtilsSeglr.h"

namespace ivpico
{
namespace helm
{
namespace
{
constexpr double degrees(double radians) { return radians * 180.0 / M_PI; }
constexpr double radians(double degrees) { return degrees * M_PI / 180.0; }

void set_object_fields(ivp::interface::Visualization& v, const XYObject& object)
{
    v.set_label(object.get_label());
    v.set_active(object.active());
    v.set_id(object.get_id());
    v.set_message(object.get_msg());
    if (object.duration_set())
        v.set_duration(object.get_duration());
    if (object.color_set("vertex"))
        v.set_vertex_color(object.get_color_str("vertex"));
    if (object.color_set("edge"))
        v.set_edge_color(object.get_color_str("edge"));
    if (object.color_set("label"))
        v.set_label_color(object.get_color_str("label"));
    if (object.color_set("fill"))
        v.set_fill_color(object.get_color_str("fill"));
    if (object.vertex_size_set())
        v.set_vertex_size(object.get_vertex_size());
    if (object.edge_size_set())
        v.set_edge_size(object.get_edge_size());
    if (object.transparency_set())
        v.set_transparency(object.get_transparency());
}

void get_object_fields(const ivp::interface::Visualization& v, XYObject& object)
{
    object.set_label(v.label());
    object.set_active(v.active());
    object.set_id(v.id());
    object.set_msg(v.message());
    if (v.time() != 0)
        object.set_time(static_cast<double>(v.time()) * 1e-6);
    if (v.has_duration())
        object.set_duration(v.duration());
    if (!v.vertex_color().empty())
        object.set_color("vertex", v.vertex_color());
    if (!v.edge_color().empty())
        object.set_color("edge", v.edge_color());
    if (!v.label_color().empty())
        object.set_color("label", v.label_color());
    if (!v.fill_color().empty())
        object.set_color("fill", v.fill_color());
    if (v.has_vertex_size())
        object.set_vertex_size(v.vertex_size());
    if (v.has_edge_size())
        object.set_edge_size(v.edge_size());
    if (v.has_transparency())
        object.set_transparency(v.transparency());
}

template <typename Shape> void add_vertices(Shape& shape, const XYSegList& segl)
{
    for (unsigned i = 0; i < segl.size(); ++i)
    {
        auto& vertex = *shape.add_vertex();
        vertex.set_x(segl.get_vx(i));
        vertex.set_y(segl.get_vy(i));
    }
}
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

std::optional<ivp::interface::Visualization> visualization(const std::string& var,
                                                           const std::string& spec)
{
    ivp::interface::Visualization v;
    if (var == "VIEW_POINT")
    {
        const XYPoint point = string2Point(spec);
        if (!point.valid())
            return std::nullopt;
        set_object_fields(v, point);
        v.mutable_point()->set_x(point.x());
        v.mutable_point()->set_y(point.y());
        if (point.z() != 0)
            v.mutable_point()->set_z(point.z());
    }
    else if (var == "VIEW_SEGLIST")
    {
        const XYSegList segl = string2SegList(spec);
        if (!segl.valid())
            return std::nullopt;
        set_object_fields(v, segl);
        add_vertices(*v.mutable_seglist(), segl);
    }
    else if (var == "VIEW_POLYGON")
    {
        const XYPolygon poly = string2Poly(spec);
        if (!poly.valid())
            return std::nullopt;
        set_object_fields(v, poly);
        add_vertices(*v.mutable_polygon(), poly);
    }
    else if (var == "VIEW_SEGLR")
    {
        const XYSeglr seglr = string2Seglr(spec);
        if (!seglr.valid())
            return std::nullopt;
        set_object_fields(v, seglr);
        auto& out = *v.mutable_seglr();
        for (unsigned i = 0; i < seglr.size(); ++i)
        {
            auto& vertex = *out.add_vertex();
            vertex.set_x(seglr.getVX(i));
            vertex.set_y(seglr.getVY(i));
        }
        out.set_ray_angle(seglr.getRayAngle());
        out.set_ray_length(seglr.getRayLen());
        out.set_head_size(seglr.getHeadSize());
    }
    else
        return std::nullopt;
    return v;
}

std::optional<ivp::interface::OffboardMessage> offboard_message(const std::string& spec)
{
    const NodeMessage msg = string2NodeMessage(spec);
    if (!msg.valid())
        return std::nullopt;

    ivp::interface::OffboardMessage out;
    out.set_source_node(msg.getSourceNode());
    out.set_destination_node(msg.getDestNode());
    out.set_destination_group(msg.getDestGroup());
    out.set_key(msg.getVarName());
    if (tokStringParse(spec, "double_val", ',', '=').empty())
        out.set_sval(msg.getStringVal());
    else
        out.set_dval(msg.getDoubleVal());
    out.set_source_app(msg.getSourceApp());
    out.set_source_behavior(msg.getSourceBehavior());
    out.set_color(msg.getColor());
    out.set_message_id(msg.getMessageID());
    out.set_ack_requested(msg.getAckRequested());
    return out;
}

std::pair<std::string, std::string> view_posting(const ivp::interface::Visualization& vis)
{
    if (vis.has_point())
    {
        XYPoint point(vis.point().x(), vis.point().y(), vis.point().z());
        get_object_fields(vis, point);
        return {"VIEW_POINT", point.get_spec()};
    }
    if (vis.has_polygon())
    {
        XYPolygon poly;
        for (const auto& v : vis.polygon().vertex()) poly.add_vertex(v.x(), v.y(), false);
        poly.determine_convexity();
        get_object_fields(vis, poly);
        return {"VIEW_POLYGON", poly.get_spec()};
    }
    if (vis.has_seglr())
    {
        XYSeglr seglr;
        for (const auto& v : vis.seglr().vertex()) seglr.addVertex(v.x(), v.y());
        seglr.setRayAngle(vis.seglr().ray_angle());
        if (vis.seglr().has_ray_length())
            seglr.setRayLen(vis.seglr().ray_length());
        if (vis.seglr().has_head_size())
            seglr.setHeadSize(vis.seglr().head_size());
        get_object_fields(vis, seglr);
        return {"VIEW_SEGLR", seglr.get_spec()};
    }
    XYSegList segl;
    for (const auto& v : vis.seglist().vertex()) segl.add_vertex(v.x(), v.y());
    get_object_fields(vis, segl);
    return {"VIEW_SEGLIST", segl.get_spec()};
}

std::string node_message_spec(const ivp::interface::OffboardMessage& msg)
{
    NodeMessage out;
    out.setSourceNode(msg.source_node());
    out.setDestNode(msg.destination_node());
    out.setDestGroup(msg.destination_group());
    out.setVarName(msg.key());
    if (msg.value_case() == ivp::interface::OffboardMessage::kDval)
        out.setDoubleVal(msg.dval());
    else
        out.setStringVal(msg.sval());
    out.setSourceApp(msg.source_app());
    out.setSourceBehavior(msg.source_behavior());
    if (!msg.color().empty())
        out.setColor(msg.color());
    out.setMessageID(msg.message_id());
    if (msg.ack_requested())
        out.setAckRequested();
    return out.getSpec();
}

} // namespace helm
} // namespace ivpico
