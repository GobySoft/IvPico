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
