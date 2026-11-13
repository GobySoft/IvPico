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

#include "contact_manager.h"

#include <cmath>
#include <cstdlib>

#include "CPAEngine.h"
#include "LinearExtrapolator.h"
#include "MBUtils.h"
#include "MacroUtils.h"
#include "NodeRecordUtils.h"
#include "VarDataPairUtils.h"

namespace ivpico
{
namespace helm
{
ContactManager::ContactManager(const ivp::interface::HelmConfig::ContactManager& cfg,
                               const std::string& ownship,
                               std::vector<std::string>& config_warnings)
    : ownship_(ownship)
{
    auto warn_unless = [&](bool ok, const std::string& what)
    {
        if (!ok)
            config_warnings.push_back("Bad contact_manager " + what);
    };

    filter_set_.addIgnoreName(ownship_);
    for (const auto& v : cfg.ignore_name()) warn_unless(filter_set_.addIgnoreName(v), "ignore_name: " + v);
    for (const auto& v : cfg.match_name()) warn_unless(filter_set_.addMatchName(v), "match_name: " + v);
    for (const auto& v : cfg.ignore_group())
        warn_unless(filter_set_.addIgnoreGroup(v), "ignore_group: " + v);
    for (const auto& v : cfg.match_group()) warn_unless(filter_set_.addMatchGroup(v), "match_group: " + v);
    for (const auto& v : cfg.ignore_type()) warn_unless(filter_set_.addIgnoreType(v), "ignore_type: " + v);
    for (const auto& v : cfg.match_type()) warn_unless(filter_set_.addMatchType(v), "match_type: " + v);
    for (const auto& v : cfg.ignore_region())
        warn_unless(filter_set_.addIgnoreRegion(v), "ignore_region: " + v);
    for (const auto& v : cfg.match_region())
        warn_unless(filter_set_.addMatchRegion(v), "match_region: " + v);
    filter_set_.setStrictIgnore(cfg.strict_ignore());
    if (!cfg.ownship_group().empty())
        filter_set_.setOwnshipGroup(cfg.ownship_group());
    if (!cfg.ownship_type().empty())
        filter_set_.setOwnshipType(cfg.ownship_type());

    if (cfg.has_contact_max_age())
    {
        warn_unless(cfg.contact_max_age() > 0, "contact_max_age");
        if (cfg.contact_max_age() > 0)
            contact_max_age_ = cfg.contact_max_age();
    }
    if (cfg.has_reject_range())
        reject_range_ = cfg.reject_range() > 0 ? cfg.reject_range() : -1;
    if (cfg.has_max_contacts())
        max_contacts_ = cfg.max_contacts();
    if (cfg.has_decay_start() || cfg.has_decay_end())
    {
        const double start = cfg.has_decay_start() ? cfg.decay_start() : decay_start_;
        const double end = cfg.has_decay_end() ? cfg.decay_end() : decay_end_;
        warn_unless(start >= 0 && start <= end, "decay");
        if (start >= 0 && start <= end)
            decay_start_ = start, decay_end_ = end;
    }
    hold_alerts_for_helm_ = cfg.hold_alerts_for_helm();
    alert_verbose_ = cfg.alert_verbose();

    disable_var_ = cfg.disable_var();
    enable_var_ = cfg.enable_var();
    expunge_var_ = cfg.expunge_var();
    auto add_flags = [&](std::vector<VarDataPair>& flags, const auto& specs, const char* what)
    {
        for (const auto& spec : specs) warn_unless(addVarDataPairOnString(flags, spec), what + (": " + spec));
    };
    add_flags(able_flags_, cfg.able_flag(), "able_flag");
    add_flags(disable_flags_, cfg.disable_flag(), "disable_flag");
    add_flags(enable_flags_, cfg.enable_flag(), "enable_flag");
    early_warning_time_ = cfg.early_warning_time() > 0 ? cfg.early_warning_time() : -1;
    add_flags(early_warning_flags_, cfg.early_warning_flag(), "early_warning_flag");
    add_flags(cease_warning_flags_, cfg.cease_warning_flag(), "cease_warning_flag");
    add_flags(retire_flags_, cfg.retire_flag(), "retire_flag");

    for (const auto& alert : cfg.alert()) warn_unless(handle_alert(alert, "general"), "alert: " + alert);
    for (const auto& [id, alert] : alerts_)
        warn_unless(alert.valid(), "alert (incomplete): " + id);

    // retirement here does the culling
    ledger_.setMaxContacts(0);
}

bool ContactManager::handle_mail(const std::string& var, const std::string& value,
                                 const std::string& source)
{
    if (var == "NODE_REPORT")
        handle_node_record(string2NodeRecord(value));
    else if (var == "BCM_ALERT_REQUEST")
        handle_alert(value, source);
    else if (!disable_var_.empty() && var == disable_var_)
        handle_mod_enable_contact(value, "disable");
    else if (!enable_var_.empty() && var == enable_var_)
        handle_mod_enable_contact(value, "enable");
    else if (!expunge_var_.empty() && var == expunge_var_)
        handle_mod_enable_contact(value, "expunge");
    else
        return false;
    return true;
}

void ContactManager::handle_node_record(const NodeRecord& record)
{
    const std::string vname = record.getName();
    if (vname.empty() || tolower(vname) == tolower(ownship_))
        return;

    // a contact that fails the filter now is retired on the next iteration,
    // even if it passed before (e.g. it left a match region)
    if (!filter_set_.filterCheck(record, osx_, osy_))
    {
        filtered_vnames_.insert(vname);
        return;
    }

    const bool known = ledger_.hasVName(vname);
    std::string whynot;
    if (ledger_.processNodeRecord(record, whynot) != vname)
        return;

    if (!known)
    {
        ranges_actual_[vname] = 0;
        ranges_extrap_[vname] = 0;
        cpa_[vname] = 0;
        roc_[vname] = 0;
        par_.addVehicle(vname);
        if (alert_verbose_)
            notify("ALERT_VERBOSE", "new_contact=" + vname);
    }
    contacts_retired_.remove(vname);
}

void ContactManager::set_ownship(double x, double y, double heading, double speed)
{
    osx_ = x;
    osy_ = y;
    osh_ = heading;
    osv_ = speed;
}

std::vector<ContactManager::Post> ContactManager::iterate(double time, bool helm_in_drive)
{
    curr_time_ = time;
    if (helm_in_drive)
        helm_in_drive_noted_ = true;

    ledger_.setCurrTimeUTC(time);
    ledger_.extrapolate();
    update_ranges();
    check_for_early_warnings();
    check_for_cease_warnings();
    if (!hold_alerts_for_helm_ || helm_in_drive_noted_)
        check_for_alerts();
    check_for_new_retired_contacts();

    return std::move(posts_);
}

bool ContactManager::handle_alert(const std::string& alert_str, const std::string& source)
{
    std::string alert_id = tokStringParse(alert_str, "id", ',', '=');
    if (alert_id.empty())
        alert_id = "no_id";

    if (strContains(tolower(alert_str), "action=disable"))
    {
        if (alert_id != "no_id")
            alerts_disabled_.insert(alert_id);
        return true;
    }
    if (strContains(tolower(alert_str), "action=enable"))
    {
        if (alert_id != "no_id")
            alerts_disabled_.erase(alert_id);
        return true;
    }

    par_.addAlertID(alert_id);
    CMAlert& alert = alerts_[alert_id];
    if (!source.empty())
        alert.setAlertSource(source);

    std::string var, pattern;
    for (std::string component : parseStringQ(alert_str, ','))
    {
        const std::string left = tolower(biteStringX(component, '='));
        std::string right = component;
        const double dval = atof(right.c_str());
        if (isQuoted(right))
            right = stripQuotes(right);

        bool ok = true;
        if (left == "id")
            continue;
        else if (left == "var")
            var = right;
        else if (left == "val" || left == "pattern")
            pattern = right;
        else if (left == "on_flag" || left == "onflag")
            ok = alert.addAlertOnFlag(right);
        else if (left == "off_flag" || left == "offflag")
            ok = alert.addAlertOffFlag(right);
        else if (left == "alert_range" && isNumber(right))
            ok = alert.setAlertRange(dval);
        else if (left == "cpa_range" && isNumber(right))
            ok = alert.setAlertRangeFar(dval);
        else if (left == "match_name" || left == "ignore_name" || left == "match_group" ||
                 left == "ignore_group" || left == "match_type" || left == "ignore_type" ||
                 left == "match_region" || left == "ignore_region" || left == "strict_ignore")
            ok = alert.configFilter(left, right);
        else
            ok = false;

        if (!ok)
            return false;
    }

    // the older form, with separate var and val
    if (!var.empty() && !pattern.empty())
        alert.addAlertOnFlag(var + "=" + pattern);
    return true;
}

void ContactManager::handle_mod_enable_contact(const std::string& str, const std::string& action)
{
    std::string contact_id, msg;
    if (!strContains(str, "=") && !strContains(str, ","))
        msg = "contact=" + str;
    else
    {
        contact_id = tokStringParse(str, "contact");
        if (!contact_id.empty())
            msg = "contact=" + contact_id;
        else
        {
            const std::string vsource = tokStringParse(str, "vsource");
            if (!vsource.empty())
                msg = "vsource=" + vsource;
        }
    }
    if (msg.empty())
        return;

    notify("BHV_ABLE_FILTER", msg + ",action=" + action);

    if (contact_id.empty())
        return;
    if (action == "enable")
    {
        post_flags(able_flags_);
        post_flags(enable_flags_);
    }
    else if (action == "disable")
    {
        post_flags(able_flags_);
        post_flags(disable_flags_);
    }
}

void ContactManager::update_ranges()
{
    // ten hours, so CPA is the closest the two tracks ever get
    const double cpa_time = 36000;

    for (const auto& contact : ledger_.getVNames())
    {
        double cnx = ledger_.getX(contact);
        double cny = ledger_.getY(contact);
        const double cnh = ledger_.getHeading(contact);
        const double cns = ledger_.getSpeed(contact);
        const double cnt = ledger_.getUTC(contact);

        const double range_actual = hypot(osx_ - cnx, osy_ - cny);
        ranges_actual_[contact] = range_actual;

        LinearExtrapolator linex;
        linex.setDecay(decay_start_, decay_end_);
        linex.setPosition(cnx, cny, cns, cnh, cnt);
        double extrap_x = cnx, extrap_y = cny;
        double range_extrap = range_actual;
        if (linex.getPosition(extrap_x, extrap_y, curr_time_))
        {
            cnx = extrap_x;
            cny = extrap_y;
            range_extrap = hypot(osx_ - cnx, osy_ - cny);
        }
        ranges_extrap_[contact] = range_extrap;

        CPAEngine engine(cny, cnx, cnh, cns, osy_, osx_);
        cpa_[contact] = engine.evalCPA(osh_, osv_, cpa_time);
        roc_[contact] = engine.evalROC(osh_, osv_);
    }
}

void ContactManager::check_for_alerts()
{
    for (const auto& contact : ledger_.getVNames())
    {
        const NodeRecord record = ledger_.getRecord(contact);
        for (const auto& [id, alert] : alerts_)
        {
            if (alerts_disabled_.count(id))
                continue;

            const bool applies = check_alert_applies(contact, id);
            std::string transition;
            if (applies && !par_.getAlertedValue(contact, id))
            {
                for (const auto& pair : alert.getAlertOnFlags()) post_alert(record, pair);
                par_.setAlertedValue(contact, id, true);
                transition = "off-->alerted";
            }
            else if (!applies && par_.getAlertedValue(contact, id))
            {
                for (const auto& pair : alert.getAlertOffFlags()) post_alert(record, pair);
                par_.setAlertedValue(contact, id, false);
                transition = "alerted-->off";
            }

            if (!alert_verbose_ || transition.empty())
                continue;
            std::string msg = "contact=" + contact + ",alert_id=" + id + "," + transition;
            msg += ",alert_range=" + doubleToStringX(alert.getAlertRange(), 1);
            msg += ",alert_range_cpa=" + doubleToStringX(alert.getAlertRangeFar(), 1);
            msg += ",range_actual=" + doubleToString(ranges_actual_[contact], 1);
            if (par_.getAlertedValue(contact, id))
                msg += ",range_cpa=" + doubleToString(cpa_[contact], 1);
            if (early_warning_time_ > 0 && transition == "off-->alerted" &&
                utc_was_warned_.count(contact))
                msg += ",after_warning=" +
                       doubleToStringX(curr_time_ - utc_was_warned_[contact], 2);
            notify("ALERT_VERBOSE", msg);
        }
    }
}

bool ContactManager::check_alert_applies(const std::string& contact, const std::string& id)
{
    const auto it = alerts_.find(id);
    if (it == alerts_.end() || !it->second.valid() || !ledger_.hasVName(contact))
        return false;
    const CMAlert& alert = it->second;

    const NodeRecord record = ledger_.getRecord(contact);
    if (curr_time_ - record.getTimeStamp() > contact_max_age_)
        return false;
    if (!alert.filterCheck(record, osx_, osy_))
        return false;

    // without a range, the alert depends only on its filters (e.g. a region)
    const double alert_range = alert.getAlertRange();
    if (alert_range > 0)
    {
        const double range = ranges_actual_[contact];
        const double range_cpa = cpa_[contact];
        if (range > alert.getAlertRangeFar())
            return false;
        if (range > alert_range && range_cpa > alert_range)
            return false;
    }
    return true;
}

double ContactManager::max_alert_range() const
{
    double max_range = -1;
    for (const auto& [id, alert] : alerts_)
        if (!alerts_disabled_.count(id))
            max_range = std::max(max_range, alert.getAlertRangeFar());
    return max_range;
}

void ContactManager::check_for_early_warnings()
{
    if (early_warning_time_ <= 0)
        return;

    for (const auto& contact : ledger_.getVNames())
    {
        if (range_was_warned_.count(contact) || !ranges_extrap_.count(contact) ||
            !roc_.count(contact))
            continue;
        const double roc = roc_[contact];
        if (roc <= 0)
            continue;

        const double alert_range = max_alert_range();
        if (alert_range <= 0)
            continue;
        if (ranges_extrap_[contact] < alert_range + roc * early_warning_time_)
        {
            post_warning_flags(early_warning_flags_, contact);
            range_was_warned_[contact] = ranges_extrap_[contact];
            utc_was_warned_[contact] = curr_time_;
        }
    }
}

void ContactManager::check_for_cease_warnings()
{
    for (const auto& contact : ledger_.getVNames())
    {
        auto it = range_was_warned_.find(contact);
        if (it == range_was_warned_.end())
            continue;
        if (ranges_extrap_[contact] > it->second * 1.05)
        {
            range_was_warned_.erase(it);
            utc_was_warned_.erase(contact);
            post_warning_flags(cease_warning_flags_, contact);
        }
    }
}

std::list<std::string> ContactManager::range_ordered_contacts() const
{
    std::vector<std::pair<double, std::string>> ordered;
    for (const auto& [contact, range] : ranges_actual_) ordered.emplace_back(range, contact);
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    std::list<std::string> contacts;
    for (const auto& [range, contact] : ordered) contacts.push_back(contact);
    return contacts;
}

void ContactManager::check_for_new_retired_contacts()
{
    const unsigned starting_count = ledger_.size();
    std::set<std::string> to_be_retired = filtered_vnames_;

    for (const auto& contact : ledger_.getVNames())
    {
        const double age = curr_time_ - ledger_.getRecord(contact).getTimeStamp();
        if (age > contact_max_age_)
            to_be_retired.insert(contact);
        // 5% beyond the reject range, so a contact near it doesn't come and go
        else if (ranges_actual_.count(contact) && reject_range_ > 0 &&
                 ranges_actual_[contact] > reject_range_ * 1.05)
            to_be_retired.insert(contact);
    }

    // too many contacts: retire at half the max age, then keep only the closest
    if (to_be_retired.size() + max_contacts_ < starting_count)
        for (const auto& contact : ledger_.getVNames())
            if (curr_time_ - ledger_.getRecord(contact).getTimeStamp() > contact_max_age_ / 2)
                to_be_retired.insert(contact);
    if (to_be_retired.size() + max_contacts_ < starting_count)
    {
        const unsigned to_cull = starting_count - (to_be_retired.size() + max_contacts_);
        unsigned culled = 0;
        const auto ordered = range_ordered_contacts();
        for (auto it = ordered.rbegin(); it != ordered.rend() && culled < to_cull; ++it)
            if (to_be_retired.insert(*it).second)
                ++culled;
    }

    for (const auto& contact : to_be_retired)
    {
        if (ledger_.hasVName(contact))
            contacts_retired_.push_front(contact);
        ledger_.clearNode(contact);
        ranges_actual_.erase(contact);
        ranges_extrap_.erase(contact);
        cpa_.erase(contact);
        roc_.erase(contact);
        par_.removeVehicle(contact);
        post_retire_flags(retire_flags_, contact);
        range_was_warned_.erase(contact);
        utc_was_warned_.erase(contact);
    }
    filtered_vnames_.clear();
}

void ContactManager::post_alert(const NodeRecord& record, const VarDataPair& pair)
{
    const std::string x = record.getStringValue("x");
    const std::string y = record.getStringValue("y");
    const std::string lat = record.getStringValue("lat");
    const std::string lon = record.getStringValue("lon");
    const std::string spd = record.getStringValue("speed");
    const std::string hdg = record.getStringValue("heading");
    const std::string dep = record.getStringValue("depth");
    const std::string utime = record.getStringValue("time");
    const std::string name = record.getName();
    const std::string type = record.getType();
    const std::string group = record.getGroup();

    // "$[SPD}" is pContactMgrV20's spelling
    auto expand = [&](std::string s, bool with_group)
    {
        s = findReplace(s, "$[X]", x);
        s = findReplace(s, "$[Y]", y);
        s = findReplace(s, "$[LAT]", lat);
        s = findReplace(s, "$[LON]", lon);
        s = findReplace(s, "$[SPD}", spd);
        s = findReplace(s, "$[HDG]", hdg);
        s = findReplace(s, "$[DEP]", dep);
        s = findReplace(s, "$[VNAME]", name);
        s = findReplace(s, "$[VTYPE]", type);
        if (with_group)
            s = findReplace(s, "$[GROUP]", group);
        s = findReplace(s, "$[UTIME]", utime);
        if (!with_group)
            s = findReplace(s, "$[UTC]", utime);
        s = findReplace(s, "%[VNAME]", name);
        s = findReplace(s, "%[VTYPE]", tolower(type));
        if (with_group)
            s = findReplace(s, "%[GROUP]", tolower(group));
        return s;
    };

    const std::string var = expand(pair.get_var(), false);
    if (!pair.is_string())
        notify(var, pair.get_ddata());
    else
        notify(var, expand(pair.get_sdata(), true));
}

void ContactManager::post_flags(const std::vector<VarDataPair>& flags)
{
    for (const auto& pair : flags)
    {
        if (!pair.is_string())
            notify(pair.get_var(), pair.get_ddata());
        else
            notify(pair.get_var(), pair.get_sdata());
    }
}

void ContactManager::post_warning_flags(const std::vector<VarDataPair>& flags,
                                        const std::string& contact)
{
    for (const auto& pair : flags)
    {
        if (!pair.is_string())
        {
            notify(pair.get_var(), pair.get_ddata());
            continue;
        }
        std::string value = macroExpand(stripBlankEnds(pair.get_sdata()), "CONTACT", contact);
        if (ranges_extrap_.count(contact))
            value = macroExpand(value, "RNG_EXT", ranges_extrap_[contact]);
        if (ranges_actual_.count(contact))
            value = macroExpand(value, "RNG", ranges_actual_[contact]);
        if (cpa_.count(contact))
            value = macroExpand(value, "CPA", cpa_[contact]);
        if (roc_.count(contact))
            value = macroExpand(value, "ROC", roc_[contact]);
        value = macroExpand(value, "UTC", curr_time_);
        if (!value.empty())
            notify(pair.get_var(), value);
    }
}

void ContactManager::post_retire_flags(const std::vector<VarDataPair>& flags,
                                       const std::string& contact)
{
    for (const auto& pair : flags)
    {
        if (!pair.is_string())
        {
            notify(pair.get_var(), pair.get_ddata());
            continue;
        }
        std::string value = macroExpand(stripBlankEnds(pair.get_sdata()), "CONTACT", contact);
        value = macroExpand(value, "UTC", curr_time_);
        if (!value.empty())
            notify(pair.get_var(), value);
    }
}

} // namespace helm
} // namespace ivpico
