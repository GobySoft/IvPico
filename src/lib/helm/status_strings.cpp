#include "status_strings.h"

namespace ivpico
{
namespace helm
{
std::string allstop_string(const ivp::interface::Status& status)
{
    switch (status.allstop())
    {
        case ivp::interface::CLEAR: return "clear";
        case ivp::interface::MANUAL_OVERRIDE: return "ManualOverride";
        case ivp::interface::BEHAVIOR_ERROR: return "BehaviorError";
        case ivp::interface::NOTHING_TO_DO: return "NothingToDo";
        case ivp::interface::NO_GOAL_BEHAVIOR: return "NoGoalBehavior";
        case ivp::interface::DISABLED_BY_STANDBY_HELM: return "DisabledByStandbyHelm";
        case ivp::interface::MISSING_DECISION_VARIABLES:
        {
            std::string s = "MissingDecVars:";
            for (int i = 0; i < status.missing_decision_variables_size(); ++i)
                s += (i ? "," : "") + status.missing_decision_variables(i);
            return s;
        }
        default: return "";
    }
}

std::string modes_string(const ivp::interface::Status& status)
{
    std::string s;
    for (const auto& mode : status.mode())
        s += (s.empty() ? "" : "#") + mode.variable() + "@" + mode.value();
    return s;
}

} // namespace helm
} // namespace ivpico
