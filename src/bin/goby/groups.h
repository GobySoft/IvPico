#ifndef IVPICO_GOBY_GROUPS_H
#define IVPICO_GOBY_GROUPS_H

#include <goby/middleware/group.h>

namespace ivpico
{
namespace goby
{
namespace groups
{
constexpr ::goby::middleware::Group navigation{"ivpico::helm::navigation"};
constexpr ::goby::middleware::Group variable{"ivpico::helm::variable"};
constexpr ::goby::middleware::Group command{"ivpico::helm::command"};

constexpr ::goby::middleware::Group solution{"ivpico::helm::solution"};
constexpr ::goby::middleware::Group posting{"ivpico::helm::posting"};
constexpr ::goby::middleware::Group status{"ivpico::helm::status"};
constexpr ::goby::middleware::Group behavior_event{"ivpico::helm::behavior_event"};
constexpr ::goby::middleware::Group visualization{"ivpico::helm::visualization"};
constexpr ::goby::middleware::Group offboard_message{"ivpico::helm::offboard_message"};
constexpr ::goby::middleware::Group life_event{"ivpico::helm::life_event"};
constexpr ::goby::middleware::Group objective_function{"ivpico::helm::objective_function"};
constexpr ::goby::middleware::Group configuration{"ivpico::helm::configuration"};
} // namespace groups
} // namespace goby
} // namespace ivpico

#endif
