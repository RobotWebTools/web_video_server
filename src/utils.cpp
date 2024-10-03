#include "web_video_server/utils.hpp"

namespace web_video_server
{

std::optional<rmw_qos_profile_t> get_qos_profile_from_name(const std::string name)
{
  if (name == "default") {
    return rmw_qos_profile_default;
  } else if (name == "system_default") {
    return rmw_qos_profile_system_default;
  } else if (name == "sensor_data") {
    return rmw_qos_profile_sensor_data;
  } else {
    return std::nullopt;
  }
}

}  // namespace web_video_server
