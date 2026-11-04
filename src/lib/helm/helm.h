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
