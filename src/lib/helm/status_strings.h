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

#ifndef IVPICO_HELM_STATUS_STRINGS_H
#define IVPICO_HELM_STATUS_STRINGS_H

#include <string>

#include "interface/ivp.pb.h"

namespace ivpico
{
namespace helm
{
// pHelmIvP's IVPHELM_ALLSTOP value
std::string allstop_string(const ivp::interface::Status& status);

// The "modes" field of pHelmIvP's IVPHELM_SUMMARY, e.g. MODE@ACTIVE:SURVEYING
std::string modes_string(const ivp::interface::Status& status);

} // namespace helm
} // namespace ivpico

#endif
