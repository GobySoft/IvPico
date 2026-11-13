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

#ifndef IVPICO_HELM_CONVERSIONS_H
#define IVPICO_HELM_CONVERSIONS_H

#include <optional>
#include <string>
#include <utility>

#include "NodeRecord.h"

#include "interface/ivp.pb.h"

namespace ivpico
{
namespace helm
{
// Radians clockwise from north to MOOS-IvP's degrees, from 0 up to 360
double heading_degrees(double radians);

// The speed entry with the given mode, or else the first entry
std::optional<double> select_speed(const openocean::Navigation& nav, openocean::SpeedMode mode);

// A contact as MOOS-IvP's contact ledger takes it (the equivalent of a NODE_REPORT)
NodeRecord node_record(const ivp::interface::Navigation& nav, openocean::SpeedMode speed_mode);

// The BHV_ABLE_FILTER string
std::string able_filter_string(const ivp::interface::Command::AbleFilter& filter);

// Sets the setpoint field for the decision on a helm domain variable
void set_desired(openocean::ControlSetpoint& desired, const std::string& domain_var, double value);

// A VIEW_POINT, VIEW_SEGLIST, VIEW_POLYGON or VIEW_SEGLR posting as a
// Visualization, or nothing for another variable or a spec that doesn't parse
std::optional<ivp::interface::Visualization> visualization(const std::string& var,
                                                           const std::string& spec);

// A NODE_MESSAGE_LOCAL posting as an OffboardMessage, or nothing if it doesn't parse
std::optional<ivp::interface::OffboardMessage> offboard_message(const std::string& spec);

// The variable (VIEW_POINT, etc.) and spec that pHelmIvP would post for a Visualization
std::pair<std::string, std::string> view_posting(const ivp::interface::Visualization& vis);

// The NODE_MESSAGE_LOCAL spec for an OffboardMessage
std::string node_message_spec(const ivp::interface::OffboardMessage& msg);

} // namespace helm
} // namespace ivpico

#endif
