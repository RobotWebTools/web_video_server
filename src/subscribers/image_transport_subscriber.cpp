// Copyright (c) 2024-2025, The Robot Web Tools Contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <mutex>
#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/node.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

#include "async_web_server_cpp/http_request.hpp"
#include "web_video_server/subscribers/image_transport_subscriber.hpp"
#include "web_video_server/utils.hpp"
#include "web_video_server/subscriber.hpp"

namespace web_video_server
{
namespace subscribers
{
ImageTransportSubscriber::ImageTransportSubscriber(rclcpp::Node::WeakPtr node)
: SubscriberBase(node, "image_transport_subscriber")
{
  auto node_ptr = lock_node();
  if (!node_ptr) {
    inactive_ = true;
    return;
  }

  const std::scoped_lock lock(subscriber_mutex_);

  if (!node_ptr->has_parameter("default_transport")) {
    node_ptr->declare_parameter("default_transport", "raw");
  }
}

ImageTransportSubscriber::~ImageTransportSubscriber()
{
  const std::scoped_lock lock(subscriber_mutex_);
  inactive_ = true;
}

// We disable deprecation warnings for image_transport API usage
// to maintain compatibility with older ROS 2 distributions.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// NOLINTBEGIN(clang-diagnostic-deprecated-declarations)

void ImageTransportSubscriber::subscribe(
  const async_web_server_cpp::HttpRequest & request,
  const std::string & topic,
  const ImageCallback & callback)
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  const std::scoped_lock lock(subscriber_mutex_);

  callback_ = callback;
  const std::string default_transport = node->get_parameter("default_transport").as_string();
  const std::string transport = request.get_query_param_value_or_default(
    "default_transport",
    default_transport);

  const std::string default_qos_profile = node->get_parameter("default_qos_profile").as_string();
  auto qos_profile_name = request.get_query_param_value_or_default(
    "qos_profile",
    default_qos_profile);

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    logger_, "Streaming topic %s with QoS profile %s", topic.c_str(),
    qos_profile_name.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name);
  if (!qos_profile) {
    qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    RCLCPP_ERROR(
      logger_,
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name.c_str());
  }

  // Create subscriber
#ifdef IMAGE_TRANSPORT_USES_OLD_API
  sub_ = image_transport::create_subscription(
    node.get(), topic,
    std::bind(&ImageTransportSubscriber::subscriber_callback, this, std::placeholders::_1),
    transport, qos_profile.value().get_rmw_qos_profile());
#else
  sub_ = image_transport::create_subscription(
    *node.get(), topic,
    std::bind(&ImageTransportSubscriber::subscriber_callback, this, std::placeholders::_1),
    transport, qos_profile.value());
#endif
}

#pragma GCC diagnostic pop
// NOLINTEND(clang-diagnostic-deprecated-declarations)

void ImageTransportSubscriber::subscriber_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_msg)
{
  const std::scoped_lock lock(subscriber_mutex_);

  if (inactive_) {return;}

  try_forward_image(input_msg);
}

std::shared_ptr<SubscriberInterface> ImageTransportSubscriberFactory::create_subscriber(
  rclcpp::Node::SharedPtr node)
{
  return std::make_shared<ImageTransportSubscriber>(node);
}

std::vector<std::string> ImageTransportSubscriberFactory::get_available_topics(
  rclcpp::Node & node)
{
  std::vector<std::string> result;
  auto topic_names_and_types = node.get_topic_names_and_types();
  for (const auto & topic_and_types : topic_names_and_types) {
    for (const auto & type : topic_and_types.second) {
      if (type == this->get_type()) {
        result.push_back(topic_and_types.first);
        break;
      }
    }
  }
  return result;
}

}  // namespace subscribers
}  // namespace web_video_server

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  web_video_server::subscribers::ImageTransportSubscriberFactory,
  web_video_server::SubscriberFactoryInterface)
