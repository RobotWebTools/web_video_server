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

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

#include "sensor_msgs/msg/image.hpp"

namespace web_video_server
{
typedef std::function<void (const sensor_msgs::msg::Image::ConstSharedPtr &)> ImageCallback;

class SubscriberInterface
{
public:
  virtual ~SubscriberInterface() {}

  /**
   * @brief Starts the streaming process
   */
  virtual void subscribe(
    const async_web_server_cpp::HttpRequest & request,
    const std::string & topic,
    const ImageCallback & callback) = 0;
};

/**
 * @brief A base class providing common functionality for Subscribers.
 */
class SubscriberBase : public SubscriberInterface
{
public:
  explicit SubscriberBase(
    rclcpp::Node::WeakPtr node,
    std::string logger_name = "subscriber");

protected:
  rclcpp::Node::SharedPtr lock_node() const;

  rclcpp::Node::WeakPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex subscriber_mutex_;
  bool inactive_;

  ImageCallback callback_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

/**
 * @brief A factory interface for creating Subscriber instances.
 */
class SubscriberFactoryInterface
{
public:
  virtual ~SubscriberFactoryInterface() = default;

  /**
   * @brief Returns the type of Subscriber created by this factory.
   *
   * This should match the "type" query parameter used to select the Subscriber.
   */
  virtual std::string get_type() = 0;

  /**
   * @brief Creates a new Subscriber instance.
   * @param request The HTTP request that initiated the Subscriber.
   * @param node The ROS2 node to use for subscribing to topics.
   * @return A shared pointer to the created Subscriber instance.
   */
  virtual std::shared_ptr<SubscriberInterface> create_subscriber(
    rclcpp::Node::SharedPtr node) = 0;

  /**
   * @brief Returns a list of available topics that can be streamed by this subscriber.
   * @param node The ROS2 node to use for discovering topics.
   * @return A vector of topic names.
   */
  virtual std::vector<std::string> get_available_topics(rclcpp::Node & node) = 0;
};

} // end namespace web_video_server
