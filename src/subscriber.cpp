// Copyright (c) 2014, Worcester Polytechnic Institute
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

#include "web_video_server/subscriber.hpp"

#include <vector>
#include <sstream>
#include <string>
#include <utility>

#include "rclcpp/node.hpp"
#include "rclcpp/logging.hpp"

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"

namespace web_video_server
{

SubscriberBase::SubscriberBase(
  rclcpp::Node::WeakPtr node,
  std::string logger_name)
: node_(node)
, logger_(node_.lock()->get_logger().get_child(logger_name))
{
}

rclcpp::Node::SharedPtr SubscriberBase::lock_node() const
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_WARN(logger_, "Unable to access node because the owning node has been destroyed");
  }
  return node;
}

void SubscriberBase::subscribe(
  const async_web_server_cpp::HttpRequest & request, 
  const std::string & topic,
  const ImageCallback & callback)
{
  std::scoped_lock lock(subscriber_mutex_);

  auto node = lock_node();
  if (!node) {
    return;
  }

  callback_ = callback;
  auto qos_profile_name = request.get_query_param_value_or_default("qos_profile", "default");

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    logger_, "Streaming topic %s with QoS profile %s", topic.c_str(),
    qos_profile_name.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
     logger_, "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name.c_str());
  }

  rclcpp::QoS qos = rclcpp::QoS(
  rclcpp::QoSInitialization(qos_profile.value().history, 1),
  qos_profile.value());

  // Create subscriber
  sub_ = node->create_subscription<sensor_msgs::msg::Image>(topic, qos,
    std::bind(&SubscriberBase::subscriberCallback, this, std::placeholders::_1));
}

void SubscriberBase::subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr &input_msg)
{
  std::scoped_lock lock(subscriber_mutex_);
  callback_(input_msg);
}

std::vector<std::string> SubscriberFactoryInterface::get_available_topics(
  rclcpp::Node & /* node */)
{
  return {};
}

}  // namespace web_video_server
