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

#include "web_video_server/streamers/ros_compressed_streamer.hpp"

#include <chrono>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <boost/asio/buffer.hpp>
#include <boost/system/system_error.hpp>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_reply.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "web_video_server/streamer.hpp"
#include "web_video_server/streamers/jpeg_streamers.hpp"
#include "web_video_server/utils.hpp"

namespace web_video_server
{
namespace streamers
{

namespace
{

using sensor_msgs::msg::CompressedImage;

rclcpp::QoS make_compressed_qos(
  const std::string & compressed_topic,
  const std::string & qos_profile_name,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(
    logger, "Streaming topic %s with QoS profile %s",
    compressed_topic.c_str(), qos_profile_name.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name);
  if (!qos_profile) {
    RCLCPP_ERROR(
      logger,
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name.c_str());
    qos_profile = rmw_qos_profile_default;
  }

  return rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.value().history, 1),
    qos_profile.value());
}

std::optional<std::string> resolve_content_type(
  const std::string & format, const rclcpp::Logger & logger)
{
  if (format.find("jpeg") != std::string::npos || format.find("jpg") != std::string::npos) {
    return std::string("image/jpeg");
  }
  if (format.find("png") != std::string::npos) {
    return std::string("image/png");
  }

  RCLCPP_WARN(
    logger, "Unknown ROS compressed image format: %s",
    format.c_str());
  return std::nullopt;
}

template<typename CallbackT>
rclcpp::Subscription<CompressedImage>::SharedPtr create_compressed_image_subscription(
  const std::string & topic,
  const std::string & qos_profile_name,
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Logger & logger,
  CallbackT && callback)
{
  const std::string compressed_topic = topic + "/compressed";
  const auto qos = make_compressed_qos(compressed_topic, qos_profile_name, logger);
  return node->create_subscription<CompressedImage>(
    compressed_topic, qos, std::forward<CallbackT>(callback));
}

bool has_compressed_topic(rclcpp::Node & node, const std::string & topic)
{
  const auto compressed_topic_name = topic + "/compressed";
  const auto tnat = node.get_topic_names_and_types();
  for (const auto & topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      continue;
    }
    const auto & topic_name = topic_and_types.first;
    if (topic_name == compressed_topic_name ||
      (topic_name.rfind('/') == 0 && topic_name.substr(1) == compressed_topic_name))
    {
      return true;
    }
  }
  return false;
}

std::vector<std::string> collect_compressed_topics(rclcpp::Node & node)
{
  std::vector<std::string> result;
  const auto tnat = node.get_topic_names_and_types();
  for (const auto & topic_and_types : tnat) {
    for (const auto & type : topic_and_types.second) {
      if (type == "sensor_msgs/msg/CompressedImage") {
        std::string topic_name = topic_and_types.first;
        if (topic_name.size() > 11 &&
          topic_name.substr(topic_name.size() - 11) == "/compressed")
        {
          topic_name = topic_name.substr(0, topic_name.size() - 11);
          result.push_back(topic_name);
        }
      }
    }
  }
  return result;
}

}  // namespace

RosCompressedStreamer::RosCompressedStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
: StreamerBase(request, connection, node, "ros_compressed_streamer"), stream_(connection)
{
  stream_.send_initial_header();
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

RosCompressedStreamer::~RosCompressedStreamer()
{
  this->inactive_ = true;
  std::scoped_lock lock(send_mutex_);  // protects send_image.
}

void RosCompressedStreamer::start()
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  image_sub_ = create_compressed_image_subscription(
    topic_, qos_profile_name_, node, logger_,
    std::bind(&RosCompressedStreamer::image_callback, this, std::placeholders::_1));
}

void RosCompressedStreamer::restream_frame(std::chrono::duration<double> max_age)
{
  if (inactive_ || (last_msg == 0)) {
    return;
  }

  if (last_frame_ + max_age < std::chrono::steady_clock::now()) {
    std::scoped_lock lock(send_mutex_);
    // don't update last_frame, it may remain an old value.
    send_image(last_msg, std::chrono::steady_clock::now());
  }
}

void RosCompressedStreamer::send_image(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg,
  const std::chrono::steady_clock::time_point & time)
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  auto content_type = resolve_content_type(msg->format, logger_);
  if (!content_type) {
    return;
  }

  try {
    stream_.send_part(time, *content_type, boost::asio::buffer(msg->data), msg);
  } catch (boost::system::system_error & e) {
    // happens when client disconnects
    RCLCPP_DEBUG(logger_, "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (std::exception & e) {
    auto & clk = *node->get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (...) {
    auto & clk = *node->get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "exception");
    inactive_ = true;
    return;
  }
}


void RosCompressedStreamer::image_callback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  std::scoped_lock lock(send_mutex_);  // protects last_msg and last_frame
  last_msg = msg;
  last_frame_ = std::chrono::steady_clock::now();
  send_image(last_msg, last_frame_);
}


std::shared_ptr<StreamerInterface> RosCompressedStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
{
  auto node_locked = node.lock();
  if (!node_locked) {
    RCLCPP_WARN(
      rclcpp::get_logger("web_video_server.RosCompressedStreamerFactory"),
      "Cannot create ROS compressed streamer because the node has expired");
    return nullptr;
  }

  std::string topic = request.get_query_param_value_or_default("topic", "");
  if (!has_compressed_topic(*node_locked, topic)) {
    RCLCPP_WARN(
      node_locked->get_logger().get_child("RosCompressedStreamerFactory"),
      "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
    return std::make_shared<MjpegStreamer>(request, connection, node);
  }

  return std::make_shared<RosCompressedStreamer>(request, connection, node);
}

std::vector<std::string> RosCompressedStreamerFactory::get_available_topics(
  rclcpp::Node & node)
{
  return collect_compressed_topics(node);
}

RosCompressedSnapshotStreamer::RosCompressedSnapshotStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::WeakPtr node)
: StreamerBase(request, connection, node, "ros_compressed_snapshot_streamer")
{
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

RosCompressedSnapshotStreamer::~RosCompressedSnapshotStreamer()
{
  this->inactive_ = true;
}

void RosCompressedSnapshotStreamer::start()
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  image_sub_ = create_compressed_image_subscription(
    topic_, qos_profile_name_, node, logger_,
    std::bind(&RosCompressedSnapshotStreamer::image_callback, this, std::placeholders::_1));
}

void RosCompressedSnapshotStreamer::restream_frame(std::chrono::duration<double>/* max_age */)
{
  // no-op, snapshot streamer doesn't restream frames
}

void RosCompressedSnapshotStreamer::send_image(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg,
  const std::chrono::steady_clock::time_point & time)
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  auto content_type = resolve_content_type(msg->format, node->get_logger());
  if (!content_type) {
    return;
  }

  char stamp[20];
  std::snprintf(
    stamp, sizeof(stamp), "%.06lf",
    std::chrono::duration_cast<std::chrono::duration<double>>(time.time_since_epoch()).count());

  try {
    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header(
      "Cache-Control",
      "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("X-Timestamp", stamp)
    .header("Pragma", "no-cache")
    .header("Content-type", *content_type)
    .header("Access-Control-Allow-Origin", "*")
    .header("Content-Length", std::to_string(msg->data.size()))
    .write(connection_);

    connection_->write(boost::asio::buffer(msg->data), msg);
  } catch (boost::system::system_error & e) {
    // happens when client disconnects
    RCLCPP_DEBUG(logger_, "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (std::exception & e) {
    auto & clk = *node->get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (...) {
    auto & clk = *node->get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "exception");
    inactive_ = true;
    return;
  }

  image_sub_.reset();
  inactive_ = true;
}

void RosCompressedSnapshotStreamer::image_callback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  send_image(msg, std::chrono::steady_clock::now());
}

std::shared_ptr<StreamerInterface>
RosCompressedSnapshotStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
{
  auto node_locked = node.lock();
  if (!node_locked) {
    RCLCPP_WARN(
      rclcpp::get_logger("web_video_server.RosCompressedSnapshotStreamerFactory"),
      "Cannot create ROS compressed snapshot streamer because the node has expired");
    return nullptr;
  }

  std::string topic = request.get_query_param_value_or_default("topic", "");
  if (!has_compressed_topic(*node_locked, topic)) {
    RCLCPP_WARN(
      node_locked->get_logger().get_child("RosCompressedSnapshotStreamerFactory"),
      "Could not find compressed image topic for %s, falling back to jpeg", topic.c_str());
    return std::make_shared<JpegSnapshotStreamer>(request, connection, node);
  }
  return std::make_shared<RosCompressedSnapshotStreamer>(request, connection, node);
}

std::vector<std::string> RosCompressedSnapshotStreamerFactory::get_available_topics(
  rclcpp::Node & node)
{
  return collect_compressed_topics(node);
}

}  // namespace streamers
}  // namespace web_video_server

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  web_video_server::streamers::RosCompressedStreamerFactory,
  web_video_server::StreamerFactoryInterface)
PLUGINLIB_EXPORT_CLASS(
  web_video_server::streamers::RosCompressedSnapshotStreamerFactory,
  web_video_server::SnapshotStreamerFactoryInterface)
