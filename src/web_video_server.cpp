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

#include "web_video_server/web_video_server.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <sstream>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/exception/exception.hpp>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_reply.hpp"
#include "async_web_server_cpp/http_server.hpp"
#include "pluginlib/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/logging.hpp"

#include "web_video_server/streamer.hpp"

using namespace std::chrono_literals;
using namespace boost::placeholders;  // NOLINT

namespace web_video_server
{

WebVideoServer::WebVideoServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("web_video_server", options), handler_group_(
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)),
  streamer_factory_loader_("web_video_server", "web_video_server::StreamerFactoryInterface"),
  snapshot_streamer_factory_loader_("web_video_server",
    "web_video_server::SnapshotStreamerFactoryInterface")
{
  declare_parameter("port", 8080);
  declare_parameter("verbose", true);
  declare_parameter("address", "0.0.0.0");
  declare_parameter("server_threads", 1);
  declare_parameter("publish_rate", -1.0);
  declare_parameter("default_stream_type", "mjpeg");
  declare_parameter("default_snapshot_type", "jpeg");

  get_parameter("port", port_);
  get_parameter("verbose", verbose_);
  get_parameter("address", address_);
  int server_threads;
  get_parameter("server_threads", server_threads);
  get_parameter("publish_rate", publish_rate_);
  get_parameter("default_stream_type", default_stream_type_);
  get_parameter("default_snapshot_type", default_snapshot_type_);

  for (auto cls : streamer_factory_loader_.getDeclaredClasses()) {
    RCLCPP_INFO(get_logger(), "Loading streamer plugin: %s", cls.c_str());
    try {
      auto streamer = streamer_factory_loader_.createSharedInstance(cls);
      streamer_factories_[streamer->get_type()] = streamer;
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }

  for (auto cls : snapshot_streamer_factory_loader_.getDeclaredClasses()) {
    RCLCPP_INFO(get_logger(), "Loading streamer plugin: %s", cls.c_str());
    try {
      auto streamer = snapshot_streamer_factory_loader_.createSharedInstance(cls);
      snapshot_streamer_factories_[streamer->get_type()] = streamer;
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
    }
  }

  handler_group_.addHandlerForPath(
    "/",
    boost::bind(&WebVideoServer::handle_list_streams, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath(
    "/stream",
    boost::bind(&WebVideoServer::handle_stream, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath(
    "/stream_viewer",
    boost::bind(&WebVideoServer::handle_stream_viewer, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath(
    "/snapshot",
    boost::bind(&WebVideoServer::handle_snapshot, this, _1, _2, _3, _4));

  try {
    server_.reset(
      new async_web_server_cpp::HttpServer(
        address_, std::to_string(port_),
        boost::bind(&WebVideoServer::handle_request, this, _1, _2, _3, _4),
        server_threads
      )
    );
  } catch (boost::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Exception when creating the web server! %s:%d",
      address_.c_str(), port_);
    throw;
  }

  RCLCPP_INFO(get_logger(), "Waiting For connections on %s:%d", address_.c_str(), port_);

  if (publish_rate_ > 0) {
    restream_timer_ = create_wall_timer(
      1s / publish_rate_,
      [this]() {restream_frames(1s / publish_rate_);});
  }

  cleanup_timer_ = create_wall_timer(500ms, [this]() {cleanup_inactive_streams();});

  server_->run();
}

WebVideoServer::~WebVideoServer()
{
  server_->stop();
}

void WebVideoServer::restream_frames(std::chrono::duration<double> max_age)
{
  std::scoped_lock lock(streamers_mutex_);

  for (auto & streamer : streamers_) {
    streamer->restream_frame(max_age);
  }
}

void WebVideoServer::cleanup_inactive_streams()
{
  std::unique_lock lock(streamers_mutex_, std::try_to_lock);
  if (lock) {
    auto new_end = std::partition(
      streamers_.begin(), streamers_.end(),
      [](const std::shared_ptr<StreamerInterface> & streamer) {return !streamer->is_inactive();});
    if (verbose_) {
      for (auto itr = new_end; itr < streamers_.end(); ++itr) {
        RCLCPP_INFO(get_logger(), "Removed Stream: %s", (*itr)->get_topic().c_str());
      }
    }
    streamers_.erase(new_end, streamers_.end());
  }
}

bool WebVideoServer::handle_request(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, const char * begin,
  const char * end)
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Handling Request: %s", request.uri.c_str());
  }

  try {
    return handler_group_(request, connection, begin, end);
  } catch (std::exception & e) {
    RCLCPP_WARN(get_logger(), "Error Handling Request: %s", e.what());
    return false;
  }
}

bool WebVideoServer::handle_stream(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, const char * begin,
  const char * end)
{
  std::string type = request.get_query_param_value_or_default("type", default_stream_type_);
  if (streamer_factories_.find(type) != streamer_factories_.end()) {
    std::shared_ptr<StreamerInterface> streamer = streamer_factories_[type]->create_streamer(
      request, connection, weak_from_this());
    streamer->start();
    std::scoped_lock lock(streamers_mutex_);
    streamers_.push_back(streamer);
  } else {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(
      request, connection, begin, end);
  }
  return true;
}

bool WebVideoServer::handle_snapshot(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, const char * begin,
  const char * end)
{
  std::string type = request.get_query_param_value_or_default("type", default_snapshot_type_);
  if (snapshot_streamer_factories_.find(type) != snapshot_streamer_factories_.end()) {
    std::shared_ptr<StreamerInterface> streamer =
      snapshot_streamer_factories_[type]->create_streamer(
      request, connection, weak_from_this());
    streamer->start();
    std::scoped_lock lock(streamers_mutex_);
    streamers_.push_back(streamer);
  } else {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(
      request, connection, begin, end);
  }
  return true;
}

bool WebVideoServer::handle_stream_viewer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, const char * begin,
  const char * end)
{
  std::string type = request.get_query_param_value_or_default("type", default_stream_type_);
  if (streamer_factories_.find(type) != streamer_factories_.end()) {
    std::string topic = request.get_query_param_value_or_default("topic", "");

    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Content-type", "text/html;")
    .write(connection);

    std::stringstream ss;
    ss << "<html><head><title>" << topic << "</title></head><body>";
    ss << "<h1>" << topic << "</h1>";
    ss << streamer_factories_[type]->create_viewer(request);
    ss << "</body></html>";
    connection->write(ss.str());
  } else {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(
      request, connection, begin, end);
  }
  return true;
}

bool WebVideoServer::handle_list_streams(
  const async_web_server_cpp::HttpRequest & /* request */,
  async_web_server_cpp::HttpConnectionPtr connection, const char * /* begin */,
  const char * /* end */)
{
  std::map<std::string, std::vector<std::string>> topics_by_streamer_type;
  std::map<std::string, std::vector<std::string>> topics_by_snapshot_type;
  std::set<std::string> all_topics;

  for (const auto & factory_pair : streamer_factories_) {
    RCLCPP_DEBUG(get_logger(), "Getting topics from factory: %s", factory_pair.first.c_str());
    std::vector<std::string> factory_topics =
      factory_pair.second->get_available_topics(*this);
    RCLCPP_DEBUG(
      get_logger(), "Factory %s returned %zu topics",
      factory_pair.first.c_str(), factory_topics.size());
    for (const auto & topic : factory_topics) {
      RCLCPP_DEBUG(get_logger(), "  Topic: %s", topic.c_str());
      topics_by_streamer_type[factory_pair.first].push_back(topic);
      all_topics.insert(topic);
    }
  }

  for (const auto & factory_pair : snapshot_streamer_factories_) {
    RCLCPP_DEBUG(get_logger(), "Getting topics from factory: %s", factory_pair.first.c_str());
    std::vector<std::string> factory_topics =
      factory_pair.second->get_available_topics(*this);
    RCLCPP_DEBUG(
      get_logger(), "Factory %s returned %zu topics",
      factory_pair.first.c_str(), factory_topics.size());
    for (const auto & topic : factory_topics) {
      RCLCPP_DEBUG(get_logger(), "  Topic: %s", topic.c_str());
      topics_by_snapshot_type[factory_pair.first].push_back(topic);
      all_topics.insert(topic);
    }
  }

  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
  .header("Connection", "close")
  .header("Server", "web_video_server")
  .header(
    "Cache-Control",
    "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
  .header("Pragma", "no-cache").header("Content-type", "text/html;").write(connection);

  connection->write(
    "<html>"
    "<head><title>ROS Streamable Topic List</title></head>"
    "<body><h1>Available ROS Topics for streaming:</h1>");
  connection->write("<ul>");
  for (const std::string & topic : all_topics) {
    std::vector<std::string> available_stream_viewers;
    std::vector<std::string> available_streams;
    std::vector<std::string> available_snapshots;

    for (const auto & factory_pair : topics_by_streamer_type) {
      const auto & type = factory_pair.first;
      const auto & topics = factory_pair.second;
      if (std::find(topics.begin(), topics.end(), topic) != topics.end()) {
        available_stream_viewers.push_back(
          "<a href=\"/stream_viewer?topic=" + topic +
          "&type=" + type + "\">" + type + "</a>");
        available_streams.push_back(
          "<a href=\"/stream?topic=" + topic +
          "&type=" + type + "\">" + type + "</a>");
      }
    }

    for (const auto & factory_pair : topics_by_snapshot_type) {
      const auto & type = factory_pair.first;
      const auto & topics = factory_pair.second;
      if (std::find(topics.begin(), topics.end(), topic) != topics.end()) {
        available_snapshots.push_back(
          "<a href=\"/snapshot?topic=" + topic +
          "&type=" + type + "\">" + type + "</a>");
      }
    }

    connection->write("<li>");
    connection->write(topic);
    connection->write("<ul>");
    if (!available_streams.empty()) {
      connection->write("<li>");
      connection->write("<a href=\"/stream_viewer?topic=" + topic + "\">");
      connection->write("Stream Viewer</a> (");
      connection->write(boost::algorithm::join(available_stream_viewers, ", "));
      connection->write(")");
      connection->write("</li>");
      connection->write("<li>");
      connection->write("<a href=\"/stream?topic=" + topic + "\">");
      connection->write("Stream</a> (");
      connection->write(boost::algorithm::join(available_streams, ", "));
      connection->write(")");
      connection->write("</li>");
    }
    if (!available_snapshots.empty()) {
      connection->write("<li>");
      connection->write("<a href=\"/snapshot?topic=" + topic + "\">");
      connection->write("Snapshot</a> (");
      connection->write(boost::algorithm::join(available_snapshots, ", "));
      connection->write(")");
      connection->write("</li>");
    }
    connection->write("</ul>");
    connection->write("</li>");
  }
  connection->write("</ul></body></html>");
  return true;
}

}  // namespace web_video_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(web_video_server::WebVideoServer)
