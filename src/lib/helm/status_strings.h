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
