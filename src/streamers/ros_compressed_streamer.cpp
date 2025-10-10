// Copyright (c) 2014, Worcester Polytechnic Institute
// Copyright (c) 2024, The Robot Web Tools Contributors
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
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <boost/asio/buffer.hpp>
#include <boost/system/system_error.hpp>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_reply.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "web_video_server/base_image_streamer.hpp"
#include "web_video_server/utils.hpp"
#include "web_video_server/streamers/jpeg_streamers.hpp"

namespace web_video_server_streamers
{

RosCompressedStreamer::RosCompressedStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node)
: BaseImageStreamer(request, connection, node), stream_(connection)
{
  stream_.sendInitialHeader();
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

RosCompressedStreamer::~RosCompressedStreamer()
{
  this->inactive_ = true;
  std::scoped_lock lock(send_mutex_);  // protects sendImage.
}

void RosCompressedStreamer::start()
{
  const std::string compressed_topic = topic_ + "/compressed";

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    node_->get_logger(), "Streaming topic %s with QoS profile %s",
    compressed_topic.c_str(), qos_profile_name_.c_str());
  auto qos_profile = web_video_server::get_qos_profile_from_name(qos_profile_name_);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name_.c_str());
  }

  // Create subscriber
  const auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.value().history, 1),
    qos_profile.value());
  image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    compressed_topic, qos,
    std::bind(&RosCompressedStreamer::imageCallback, this, std::placeholders::_1));
}

void RosCompressedStreamer::restreamFrame(std::chrono::duration<double> max_age)
{
  if (inactive_ || (last_msg == 0)) {
    return;
  }

  if (last_frame_ + max_age < std::chrono::steady_clock::now()) {
    std::scoped_lock lock(send_mutex_);
    // don't update last_frame, it may remain an old value.
    sendImage(last_msg, std::chrono::steady_clock::now());
  }
}

void RosCompressedStreamer::sendImage(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg,
  const std::chrono::steady_clock::time_point & time)
{
  try {
    std::string content_type;
    if (msg->format.find("jpeg") != std::string::npos ||
      msg->format.find("jpg") != std::string::npos)
    {
      content_type = "image/jpeg";
    } else if (msg->format.find("png") != std::string::npos) {
      content_type = "image/png";
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "Unknown ROS compressed image format: %s",
        msg->format.c_str());
      return;
    }

    stream_.sendPart(time, content_type, boost::asio::buffer(msg->data), msg);
  } catch (boost::system::system_error & e) {
    // happens when client disconnects
    RCLCPP_DEBUG(node_->get_logger(), "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (std::exception & e) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (...) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception");
    inactive_ = true;
    return;
  }
}


void RosCompressedStreamer::imageCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  std::scoped_lock lock(send_mutex_);  // protects last_msg and last_frame
  last_msg = msg;
  last_frame_ = std::chrono::steady_clock::now();
  sendImage(last_msg, last_frame_);
}


std::shared_ptr<web_video_server::BaseImageStreamer> RosCompressedStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::SharedPtr node)
{
  std::string topic = request.get_query_param_value_or_default("topic", "");
  std::string compressed_topic_name = topic + "/compressed";
  auto tnat = node->get_topic_names_and_types();
  bool did_find_compressed_topic = false;
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // skip over topics with more than one type
      continue;
    }
    auto & topic_name = topic_and_types.first;
    if (topic_name == compressed_topic_name ||
      (topic_name.find("/") == 0 && topic_name.substr(1) == compressed_topic_name))
    {
      did_find_compressed_topic = true;
      break;
    }
  }
  if (!did_find_compressed_topic) {
    RCLCPP_WARN(
        node->get_logger().get_child("RosCompressedStreamerFactory"),
        "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
    return std::make_shared<MjpegStreamer>(request, connection, node);
  }

  return std::make_shared<RosCompressedStreamer>(request, connection, node);
}

std::vector<std::string> RosCompressedStreamerFactory::get_available_topics(
  rclcpp::Node::SharedPtr node)
{
  std::vector<std::string> result;
  auto tnat = node->get_topic_names_and_types();
  for (auto topic_and_types : tnat) {
    for (auto & type : topic_and_types.second) {
      if (type == "sensor_msgs/msg/CompressedImage") {
        std::string topic_name = topic_and_types.first;
        if (topic_name.size() > 11 && topic_name.substr(topic_name.size() - 11) == "/compressed") {
          topic_name = topic_name.substr(0, topic_name.size() - 11);
        }
        result.push_back(topic_name);
      }
    }
  }
  return result;
}

RosCompressedSnapshotStreamer::RosCompressedSnapshotStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node)
: web_video_server::BaseImageStreamer(request, connection, node)
{
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

RosCompressedSnapshotStreamer::~RosCompressedSnapshotStreamer()
{
  this->inactive_ = true;
}

void RosCompressedSnapshotStreamer::start()
{
  const std::string compressed_topic = topic_ + "/compressed";

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    node_->get_logger(), "Streaming topic %s with QoS profile %s",
    compressed_topic.c_str(), qos_profile_name_.c_str());
  auto qos_profile = web_video_server::get_qos_profile_from_name(qos_profile_name_);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name_.c_str());
  }

  // Create subscriber
  const auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.value().history, 1),
    qos_profile.value());
  image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    compressed_topic, qos,
      std::bind(&RosCompressedSnapshotStreamer::imageCallback, this, std::placeholders::_1)
  );
}

void RosCompressedSnapshotStreamer::restreamFrame(std::chrono::duration<double>/* max_age */)
{
  // no-op, snapshot streamer doesn't restream frames
}

void RosCompressedSnapshotStreamer::sendImage(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg,
  const std::chrono::steady_clock::time_point & time)
{
  std::string content_type;
  if (msg->format.find("jpeg") != std::string::npos ||
    msg->format.find("jpg") != std::string::npos)
  {
    content_type = "image/jpeg";
  } else if (msg->format.find("png") != std::string::npos) {
    content_type = "image/png";
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "Unknown ROS compressed image format: %s",
      msg->format.c_str());
    return;
  }

  char stamp[20];
  snprintf(
    stamp, sizeof(stamp), "%.06lf",
    std::chrono::duration_cast<std::chrono::duration<double>>(time.time_since_epoch()).count());
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
  .header("Connection", "close")
  .header("Server", "web_video_server")
  .header(
    "Cache-Control",
    "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
  .header("X-Timestamp", stamp)
  .header("Pragma", "no-cache")
  .header("Content-type", content_type)
  .header("Access-Control-Allow-Origin", "*")
  .header("Content-Length", std::to_string(msg->data.size()))
  .write(connection_);
  connection_->write(boost::asio::buffer(msg->data), msg);

  image_sub_.reset();
  inactive_ = true;
}

void RosCompressedSnapshotStreamer::imageCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  sendImage(msg, std::chrono::steady_clock::now());
}

std::shared_ptr<web_video_server::BaseImageStreamer>
RosCompressedSnapshotStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::SharedPtr node)
{
  std::string topic = request.get_query_param_value_or_default("topic", "");
  std::string compressed_topic_name = topic + "/compressed";
  auto tnat = node->get_topic_names_and_types();
  bool did_find_compressed_topic = false;
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // skip over topics with more than one type
      continue;
    }
    auto & topic_name = topic_and_types.first;
    if (topic_name == compressed_topic_name ||
      (topic_name.find("/") == 0 && topic_name.substr(1) == compressed_topic_name))
    {
      did_find_compressed_topic = true;
      break;
    }
  }
  if (!did_find_compressed_topic) {
    RCLCPP_WARN(
        node->get_logger().get_child("RosCompressedSnapshotStreamerFactory"),
        "Could not find compressed image topic for %s, falling back to jpeg", topic.c_str());
    return std::make_shared<JpegSnapshotStreamer>(request, connection, node);
  }
  return std::make_shared<RosCompressedSnapshotStreamer>(request, connection, node);
}

}  // namespace web_video_server_streamers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  web_video_server_streamers::RosCompressedStreamerFactory,
  web_video_server::BaseImageStreamerFactory)
PLUGINLIB_EXPORT_CLASS(
  web_video_server_streamers::RosCompressedSnapshotStreamerFactory,
  web_video_server::BaseSnapshotStreamerFactory)
