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

// goby_moos_gateway plugin that gives a MOOS community what pHelmIvP and its
// navigation would post (NAV_*, the helm's postings, IVPHELM_*), so MOOS-IvP
// tools such as pNodeReporter and pMarineViewer work with goby_helm_ivp.

#include <goby/middleware/marshalling/protobuf.h>

#include <goby/middleware/frontseat/groups.h>
#include <goby/middleware/protobuf/frontseat_data.pb.h>
#include <goby/moos/frontseat/convert.h>
#include <goby/moos/middleware/moos_plugin_translator.h>
#include <goby/zeromq/application/multi_thread.h>

#include "groups.h"
#include "helm/status_strings.h"
#include "interface/ivp.pb.h"
#include "moos_gateway_config.pb.h"

namespace ivpico
{
namespace goby
{
class MOOSGatewayTranslation : public ::goby::moos::Translator
{
  public:
    MOOSGatewayTranslation(const ::goby::apps::moos::protobuf::GobyMOOSGatewayConfig& cfg)
        : ::goby::moos::Translator(cfg), cfg_(cfg.GetExtension(protobuf::ivpico))
    {
        using ::goby::middleware::frontseat::protobuf::NodeStatus;
        goby()
            .interprocess()
            .subscribe<::goby::middleware::frontseat::groups::node_status, NodeStatus,
                       ::goby::middleware::MarshallingScheme::PROTOBUF>(
                [this](const NodeStatus& status)
                { ::goby::moos::convert_and_publish_node_status(status, moos().comms()); });
        goby().interprocess().subscribe<groups::posting>(
            [this](const ivp::interface::Posting& posting) { post(posting); });
        goby().interprocess().subscribe<groups::status>([this](const ivp::interface::Status& status)
                                                        { post(status); });

        for (const auto& var : cfg_.to_helm())
            moos().add_trigger(var, [this](const CMOOSMsg& msg) { to_helm(msg); });
    }

  private:
    void post(const ivp::interface::Posting& posting)
    {
        if (posting.value_case() == ivp::interface::Posting::kSval)
            moos().comms().Notify(posting.key(), posting.sval());
        else
            moos().comms().Notify(posting.key(), posting.dval());
    }

    void post(const ivp::interface::Status& status)
    {
        auto& comms = moos().comms();
        comms.Notify("IVPHELM_STATE", ivp::interface::Status::State_Name(status.state()));
        comms.Notify("IVPHELM_ALLSTOP", helm::allstop_string(status));
        const std::string modes = helm::modes_string(status);
        comms.Notify("IVPHELM_SUMMARY", "modes=" + (modes.empty() ? "none" : modes));
    }

    void to_helm(const CMOOSMsg& msg)
    {
        // the helm's own postings of these variables come back from the MOOSDB
        if (msg.GetSource() == moos().comms().GetMOOSName())
            return;

        ivp::interface::Variable variable;
        variable.set_key(msg.GetKey());
        if (msg.IsString())
            variable.set_sval(msg.GetString());
        else
            variable.set_dval(msg.GetDouble());
        goby().interprocess().publish<groups::variable>(variable);
    }

    protobuf::MOOSGatewayConfig cfg_;
};
} // namespace goby
} // namespace ivpico

extern "C"
{
    void goby3_moos_gateway_load(
        goby::zeromq::MultiThreadApplication<goby::apps::moos::protobuf::GobyMOOSGatewayConfig>*
            handler)
    {
        handler->launch_thread<ivpico::goby::MOOSGatewayTranslation>();
    }

    void goby3_moos_gateway_unload(
        goby::zeromq::MultiThreadApplication<goby::apps::moos::protobuf::GobyMOOSGatewayConfig>*
            handler)
    {
        handler->join_thread<ivpico::goby::MOOSGatewayTranslation>();
    }
}
