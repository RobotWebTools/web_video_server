#pragma once

#include <string>
#include <optional>
#include "rmw/qos_profiles.h"

namespace web_video_server
{

/**
 * @brief Gets a QoS profile given an input name, if valid.
 * @param name The name of the QoS profile name.
 * @return An optional containing the matching QoS profile.
 */
std::optional<rmw_qos_profile_t> get_qos_profile_from_name(const std::string name);

}  // namespace web_video_server
