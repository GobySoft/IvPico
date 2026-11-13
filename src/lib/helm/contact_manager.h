// Copyright 2026:
//   GobySoft, LLC
// Ports logic from MOOS-IvP's pContactMgrV20 (ContactMgrV20.cpp), by Michael
// Benjamin, Dept of Mechanical Engineering, MIT.
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

#ifndef IVPICO_HELM_CONTACT_MANAGER_H
#define IVPICO_HELM_CONTACT_MANAGER_H

#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ContactLedger.h"
#include "ExFilterSet.h"
#include "NodeRecord.h"
#include "VarDataPair.h"
#include "contacts/CMAlert.h"
#include "contacts/PlatformAlertRecord.h"
#include "interface/config.pb.h"

namespace ivpico
{
namespace helm
{
// pContactMgrV20's alerts, early warnings and retirement, without its reports
class ContactManager
{
  public:
    struct Post
    {
        std::string var;
        bool is_string{false};
        double d{0};
        std::string s;
    };

    ContactManager(const ivp::interface::HelmConfig::ContactManager& cfg,
                   const std::string& ownship, std::vector<std::string>& config_warnings);

    ContactLedger& ledger() { return ledger_; }

    // Returns false if the variable isn't one the contact manager reads
    bool handle_mail(const std::string& var, const std::string& value,
                     const std::string& source = "");
    void handle_node_record(const NodeRecord& record);

    void set_ownship(double x, double y, double heading, double speed);
    std::vector<Post> iterate(double time, bool helm_in_drive);

  private:
    bool handle_alert(const std::string& alert, const std::string& source);
    void handle_mod_enable_contact(const std::string& value, const std::string& action);

    void update_ranges();
    void check_for_alerts();
    void check_for_early_warnings();
    void check_for_cease_warnings();
    void check_for_new_retired_contacts();
    bool check_alert_applies(const std::string& contact, const std::string& id);
    double max_alert_range() const;
    std::list<std::string> range_ordered_contacts() const;

    void post_alert(const NodeRecord& record, const VarDataPair& pair);
    void post_flags(const std::vector<VarDataPair>& flags);
    void post_warning_flags(const std::vector<VarDataPair>& flags, const std::string& contact);
    void post_retire_flags(const std::vector<VarDataPair>& flags, const std::string& contact);
    void notify(const std::string& var, const std::string& s) { posts_.push_back({var, true, 0, s}); }
    void notify(const std::string& var, double d) { posts_.push_back({var, false, d, {}}); }

    std::string ownship_;
    ContactLedger ledger_;
    ExFilterSet filter_set_;
    PlatformAlertRecord par_;
    std::map<std::string, CMAlert> alerts_;
    std::set<std::string> alerts_disabled_;

    double contact_max_age_{60};
    double reject_range_{2000};
    unsigned max_contacts_{500};
    double decay_start_{15};
    double decay_end_{30};
    bool hold_alerts_for_helm_{false};
    bool helm_in_drive_noted_{false};
    bool alert_verbose_{false};
    double early_warning_time_{-1};

    std::string disable_var_, enable_var_, expunge_var_;
    std::vector<VarDataPair> able_flags_, disable_flags_, enable_flags_;
    std::vector<VarDataPair> early_warning_flags_, cease_warning_flags_, retire_flags_;

    double curr_time_{0};
    double osx_{0}, osy_{0}, osh_{0}, osv_{0};

    std::map<std::string, double> ranges_actual_, ranges_extrap_, cpa_, roc_;
    std::map<std::string, double> range_was_warned_, utc_was_warned_;
    std::set<std::string> filtered_vnames_;
    std::list<std::string> contacts_retired_;

    std::vector<Post> posts_;
};

} // namespace helm
} // namespace ivpico

#endif
