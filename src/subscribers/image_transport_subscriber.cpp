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

#include "web_video_server/subscribers/image_transport_subscriber.hpp"

namespace web_video_server
{
namespace subscribers
{
ImageTransportSubscriber::ImageTransportSubscriber(rclcpp::Node::WeakPtr node)
: SubscriberBase(node, "image_transport_subscriber")
{
}

ImageTransportSubscriber::~ImageTransportSubscriber()
{
    std::scoped_lock lock(subscriber_mutex_);
}

void ImageTransportSubscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                         const std::string& topic, 
                                         const ImageCallback& callback)
{
  std::scoped_lock lock(subscriber_mutex_);
  
  auto node = lock_node();
  if (!node) {
    return;
  }

  callback_ = callback;
  std::string transport = request.get_query_param_value_or_default("transport", "raw");
  auto qos_profile_name = request.get_query_param_value_or_default("qos_profile", "default");

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    logger_, "Streaming topic %s with QoS profile %s", topic.c_str(),
    qos_profile_name.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      logger_,
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name.c_str());
  }

  const auto qos = qos_profile.value();
  
  sub_ = image_transport::create_subscription(node.get(), topic, 
             std::bind(&ImageTransportSubscriber::subscriberCallback, this, std::placeholders::_1), 
             transport, qos);
}

void ImageTransportSubscriber::subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr &input_msg)
{
  std::scoped_lock lock(subscriber_mutex_);
  
  callback_(input_msg);
}

std::shared_ptr<SubscriberInterface> ImageTransportSubscriberFactory::create_subscriber(
    rclcpp::Node::WeakPtr node)
{
  return std::make_shared<ImageTransportSubscriber>(node);
}

std::vector<std::string> ImageTransportSubscriberFactory::get_available_topics(
  rclcpp::Node & node)
{
  return std::vector<std::string>(); //TBD
}

}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  web_video_server::subscribers::ImageTransportSubscriberFactory,
  web_video_server::SubscriberFactoryInterface)