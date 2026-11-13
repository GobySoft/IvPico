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

// Replays contacts' recorded NODE_REPORTs to goby_helm_ivp, for the closed-loop
// mission tests. The recording's first IVPHELM_STATE = DRIVE lines up with the
// helm's first DRIVE, and each report goes out at its recorded offset from
// there (in warped time), restamped with the current time. The contacts don't
// react to ownship.
//
// Publishes each contact's range to ownship (frontseat NodeStatus) as a
// Variable "<NAME>_RANGE" on ivpico::test::ghost_range.

#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

#include <goby/middleware/frontseat/groups.h>
#include <goby/middleware/marshalling/protobuf.h>
#include <goby/middleware/protobuf/frontseat_data.pb.h>
#include <goby/time/system_clock.h>
#include <goby/zeromq/application/single_thread.h>

#include "ghost_config.pb.h"
#include "groups.h"
#include "interface/ivp.pb.h"

namespace ivpico
{
namespace goby
{
namespace groups
{
constexpr ::goby::middleware::Group ghost_range{"ivpico::test::ghost_range"};
}

class Ghost : public ::goby::zeromq::SingleThreadApplication<protobuf::GhostConfig>
{
  public:
    Ghost();

  private:
    struct Report
    {
        double offset;
        std::string name;
        std::map<std::string, std::string> fields;
        std::vector<std::string> keys;
    };

    void read_alog();
    void loop() override;
    double now() const;
    std::string restamp(const Report& report) const;

    std::deque<Report> reports_;
    std::optional<double> start_;
    std::map<std::string, std::pair<double, double>> contact_xy_;
};
} // namespace goby
} // namespace ivpico

int main(int argc, char* argv[]) { return goby::run<ivpico::goby::Ghost>(argc, argv); }

ivpico::goby::Ghost::Ghost()
    : ::goby::zeromq::SingleThreadApplication<protobuf::GhostConfig>(20 * boost::units::si::hertz)
{
    read_alog();

    interprocess().subscribe<groups::status>(
        [this](const ivp::interface::Status& status)
        {
            if (!start_ && status.state() == ivp::interface::Status::DRIVE)
                start_ = now();
        });

    interprocess().subscribe<::goby::middleware::frontseat::groups::node_status>(
        [this](const ::goby::middleware::frontseat::protobuf::NodeStatus& status)
        {
            for (const auto& [name, xy] : contact_xy_)
            {
                ivp::interface::Variable range;
                range.set_time(static_cast<int64_t>(now() * 1e6));
                range.set_key(name + "_RANGE");
                range.set_dval(std::hypot(status.local_fix().x() - xy.first,
                                          status.local_fix().y() - xy.second));
                interprocess().publish<groups::ghost_range>(range);
            }
        });
}

double ivpico::goby::Ghost::now() const
{
    return ::goby::time::SystemClock::now<::goby::time::SITime>().value();
}

void ivpico::goby::Ghost::read_alog()
{
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(
        popen(("xz -dc '" + cfg().alog() + "'").c_str(), "r"), pclose);
    if (!pipe)
        throw std::runtime_error("can't run xz on " + cfg().alog());
    std::string text;
    char buffer[65536];
    for (size_t n; (n = fread(buffer, 1, sizeof(buffer), pipe.get())) > 0;) text.append(buffer, n);

    const std::set<std::string> vehicles(cfg().vehicle().begin(), cfg().vehicle().end());
    std::optional<double> drive;
    std::istringstream lines(text);
    for (std::string line; std::getline(lines, line);)
    {
        std::istringstream fields(line);
        double time;
        std::string var, source, value;
        if (!(fields >> time >> var >> source))
            continue;
        std::getline(fields, value);
        const auto begin = value.find_first_not_of(" \t");
        value = begin == std::string::npos
                    ? ""
                    : value.substr(begin, value.find_last_not_of(" \t") - begin + 1);

        if (!drive && var == "IVPHELM_STATE" && value == "DRIVE")
            drive = time;
        if (var != "NODE_REPORT" || !drive)
            continue;

        Report report;
        report.offset = time - *drive;
        std::istringstream pairs(value);
        for (std::string pair; std::getline(pairs, pair, ',');)
        {
            const auto eq = pair.find('=');
            if (eq == std::string::npos)
                continue;
            const std::string key = pair.substr(0, eq);
            report.keys.push_back(key);
            report.fields[key] = pair.substr(eq + 1);
        }
        report.name = report.fields["NAME"];
        if (vehicles.count(report.name))
            reports_.push_back(report);
    }
    if (reports_.empty())
        throw std::runtime_error("no NODE_REPORTs after DRIVE for the vehicles in " + cfg().alog());
}

std::string ivpico::goby::Ghost::restamp(const Report& report) const
{
    std::string spec;
    for (const auto& key : report.keys)
    {
        // the helm places the contact from X and Y with its own datum
        if (key == "LAT" || key == "LON")
            continue;
        const std::string value = key == "TIME" ? std::to_string(now()) : report.fields.at(key);
        spec += (spec.empty() ? "" : ",") + key + "=" + value;
    }
    return spec;
}

void ivpico::goby::Ghost::loop()
{
    if (!start_)
        return;
    const double elapsed = now() - *start_;
    while (!reports_.empty() && reports_.front().offset <= elapsed)
    {
        const Report& report = reports_.front();
        ivp::interface::Variable v;
        v.set_time(static_cast<int64_t>(now() * 1e6));
        v.set_key("NODE_REPORT");
        v.set_sval(restamp(report));
        interprocess().publish<groups::variable>(v);
        contact_xy_[report.name] = {std::stod(report.fields.at("X")),
                                    std::stod(report.fields.at("Y"))};
        reports_.pop_front();
    }
}
