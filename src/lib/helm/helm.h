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

#ifndef IVPICO_HELM_HELM_H
#define IVPICO_HELM_HELM_H

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "interface/config.pb.h"
#include "interface/ivp.pb.h"

namespace ivpico
{
namespace helm
{
// Conversions between latitude/longitude (degrees) and the local x/y grid (meters)
struct Geodesy
{
    std::function<void(double lat, double lon, double& x, double& y)> to_local;
    std::function<void(double x, double y, double& lat, double& lon)> to_global;
};

// Everything the helm published on one iteration
struct Output
{
    std::optional<ivp::interface::Solution> solution;
    ivp::interface::Status status;
    std::vector<ivp::interface::Posting> postings;
    std::vector<ivp::interface::BehaviorEvent> behavior_events;
    std::vector<ivp::interface::Visualization> visualizations;
    std::vector<ivp::interface::OffboardMessage> offboard_messages;
    std::vector<ivp::interface::LifeEvent> life_events;
    std::vector<ivp::interface::ObjectiveFunction> objective_functions;
    std::optional<ivp::interface::Configuration> configuration;
};

// pHelmIvP without MOOS: inputs are queued as they arrive and applied at the
// start of the next iterate(), as pHelmIvP applies its mail.
class Helm
{
  public:
    explicit Helm(const ivp::interface::HelmConfig& cfg,
                  std::optional<Geodesy> geodesy = std::nullopt);
    ~Helm();

    void handle(const ivp::interface::Navigation& navigation);
    void handle(const ivp::interface::Variable& variable);
    void handle(const ivp::interface::Command& command);

    // time: seconds since the UNIX epoch (warped, if the caller warps time)
    const Output& iterate(double time);

    ivp::interface::Status::State state() const;
    // configuration and .bhv problems; any of them leaves the helm in MALCONFIG
    const std::vector<std::string>& config_warnings() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace helm
} // namespace ivpico

#endif
