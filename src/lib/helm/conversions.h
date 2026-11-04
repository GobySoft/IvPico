#ifndef IVPICO_HELM_CONVERSIONS_H
#define IVPICO_HELM_CONVERSIONS_H

#include <optional>
#include <string>

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

} // namespace helm
} // namespace ivpico

#endif
