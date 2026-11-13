// Copyright (c) 2026, The Robot Web Tools Contributors
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

#include "web_video_server/streamers/foxglove_compressed_image_streamer.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <map>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "foxglove_msgs/msg/compressed_image.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

#include "web_video_server/streamer.hpp"

#include <boost/asio/buffer.hpp>
#include <boost/system/system_error.hpp>

#include "async_web_server_cpp/http_reply.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"

#include "web_video_server/utils.hpp"

namespace web_video_server
{
namespace streamers
{
namespace
{

using Message = foxglove_msgs::msg::CompressedImage;
constexpr const char * KMessageType = "foxglove_msgs/msg/CompressedImage";

rclcpp::QoS make_qos(
  const std::string & topic, const std::string & profile_name, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(
    logger, "Streaming topic %s with QoS profile %s", topic.c_str(),
    profile_name.c_str());
  auto profile = get_qos_profile_from_name(profile_name);
  if (!profile) {
    RCLCPP_ERROR(logger, "Invalid QoS profile %s; using default", profile_name.c_str());
    profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  }
  auto qos = profile.value();
  qos.keep_last(1);
  return qos;
}

std::optional<std::string> content_type(const std::string & format)
{
  if (format == "jpeg" || format == "jpg") {return "image/jpeg";}
  if (format == "png") {return "image/png";}
  if (format == "webp") {return "image/webp";}
  if (format == "avif") {return "image/avif";}
  return std::nullopt;
}

bool topic_has_type(rclcpp::Node & node, const std::string & topic, const std::string & type)
{
  const auto names_and_types = node.get_topic_names_and_types();
  const auto it = names_and_types.find(topic);
  if (it == names_and_types.end()) {return false;}
  return std::find(it->second.begin(), it->second.end(), type) != it->second.end();
}

std::vector<std::string> collect_topics(rclcpp::Node & node)
{
  std::vector<std::string> result;
  for (const auto & item : node.get_topic_names_and_types()) {
    if (std::find(item.second.begin(), item.second.end(), KMessageType) != item.second.end()) {
      result.push_back(item.first);
    }
  }
  return result;
}

}  // namespace

FoxgloveCompressedImageStreamer::FoxgloveCompressedImageStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>> & subscriber_factories,
  rclcpp::Node::WeakPtr node)
: StreamerBase(
    request, connection, subscriber_factories, node, "foxglove_compressed_image_streamer"),
  stream_(connection)
{
  stream_.send_initial_header();
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

FoxgloveCompressedImageStreamer::~FoxgloveCompressedImageStreamer()
{
  inactive_ = true;
  const std::scoped_lock lock(send_mutex_);
}

void FoxgloveCompressedImageStreamer::start()
{
  auto node = lock_node();
  if (!node) {inactive_ = true; return;}
  image_sub_ = node->create_subscription<Message>(
    topic_, make_qos(topic_, qos_profile_name_, logger_),
    std::bind(&FoxgloveCompressedImageStreamer::image_callback, this, std::placeholders::_1));
}

void FoxgloveCompressedImageStreamer::restream_frame(std::chrono::duration<double> max_age)
{
  if (inactive_ || !last_msg_) {return;}
  if (last_frame_ + max_age < std::chrono::steady_clock::now()) {
    const std::scoped_lock lock(send_mutex_);
    send_image(last_msg_, std::chrono::steady_clock::now());
  }
}

void FoxgloveCompressedImageStreamer::image_callback(Message::ConstSharedPtr msg)
{
  const std::scoped_lock lock(send_mutex_);
  last_msg_ = std::move(msg);
  last_frame_ = std::chrono::steady_clock::now();
  send_image(last_msg_, last_frame_);
}

void FoxgloveCompressedImageStreamer::send_image(
  Message::ConstSharedPtr msg, const std::chrono::steady_clock::time_point & time)
{
  const auto type = content_type(msg->format);
  if (!type) {
    RCLCPP_WARN(logger_, "Unsupported Foxglove compressed image format: %s", msg->format.c_str());
    return;
  }
  try {
    stream_.send_part(time, *type, boost::asio::buffer(msg->data), msg);
  } catch (const boost::system::system_error & e) {
    RCLCPP_DEBUG(logger_, "Client disconnected: %s", e.what());
    inactive_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Could not send Foxglove compressed image: %s", e.what());
    inactive_ = true;
  }
}

std::shared_ptr<StreamerInterface> FoxgloveCompressedImageStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>> & subscriber_factories,
  rclcpp::Node::WeakPtr node)
{
  auto locked = node.lock();
  if (!locked) {return nullptr;}
  const auto topic = request.get_query_param_value_or_default("topic", "");
  if (!topic_has_type(*locked, topic, KMessageType)) {
    RCLCPP_WARN(locked->get_logger(), "%s is not %s", topic.c_str(), KMessageType);
    return nullptr;
  }
  return std::make_shared<FoxgloveCompressedImageStreamer>(
    request, connection, subscriber_factories, node);
}

std::vector<std::string> FoxgloveCompressedImageStreamerFactory::get_available_topics(
  rclcpp::Node & node,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>>/* subscriber_factories */)
{
  return collect_topics(node);
}

FoxgloveCompressedImageSnapshotStreamer::FoxgloveCompressedImageSnapshotStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>> & subscriber_factories,
  rclcpp::Node::WeakPtr node)
: StreamerBase(
    request, connection, subscriber_factories, node,
    "foxglove_compressed_image_snapshot_streamer")
{
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

FoxgloveCompressedImageSnapshotStreamer::~FoxgloveCompressedImageSnapshotStreamer()
{
  inactive_ = true;
}

void FoxgloveCompressedImageSnapshotStreamer::start()
{
  auto node = lock_node();
  if (!node) {inactive_ = true; return;}
  image_sub_ = node->create_subscription<Message>(
    topic_, make_qos(topic_, qos_profile_name_, logger_),
    std::bind(
      &FoxgloveCompressedImageSnapshotStreamer::image_callback, this, std::placeholders::_1));
}

void FoxgloveCompressedImageSnapshotStreamer::restream_frame(
  std::chrono::duration<double>/* max_age */)
{
}

void FoxgloveCompressedImageSnapshotStreamer::image_callback(Message::ConstSharedPtr msg)
{
  const auto type = content_type(msg->format);
  if (!type) {
    RCLCPP_WARN(logger_, "Unsupported Foxglove compressed image format: %s", msg->format.c_str());
    return;
  }

  try {
    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate")
    .header("Pragma", "no-cache")
    .header("Content-type", *type)
    .header("Access-Control-Allow-Origin", "*")
    .header("Content-Length", std::to_string(msg->data.size()))
    .write(connection_);
    connection_->write(boost::asio::buffer(msg->data), msg);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger_, "Snapshot connection closed: %s", e.what());
  }
  image_sub_.reset();
  inactive_ = true;
}

std::shared_ptr<StreamerInterface>
FoxgloveCompressedImageSnapshotStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>> & subscriber_factories,
  rclcpp::Node::WeakPtr node)
{
  auto locked = node.lock();
  if (!locked) {return nullptr;}
  const auto topic = request.get_query_param_value_or_default("topic", "");
  if (!topic_has_type(*locked, topic, KMessageType)) {return nullptr;}
  return std::make_shared<FoxgloveCompressedImageSnapshotStreamer>(
    request, connection, subscriber_factories, node);
}

std::vector<std::string>
FoxgloveCompressedImageSnapshotStreamerFactory::get_available_topics(
  rclcpp::Node & node,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>>/* subscriber_factories */)
{
  return collect_topics(node);
}

}  // namespace streamers
}  // namespace web_video_server

PLUGINLIB_EXPORT_CLASS(
  web_video_server::streamers::FoxgloveCompressedImageStreamerFactory,
  web_video_server::StreamerFactoryInterface)
PLUGINLIB_EXPORT_CLASS(
  web_video_server::streamers::FoxgloveCompressedImageSnapshotStreamerFactory,
  web_video_server::SnapshotStreamerFactoryInterface)
