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

#include "web_video_server/web_video_server.hpp"

#include <chrono>
#include <vector>

#include <boost/algorithm/string/predicate.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "web_video_server/ros_compressed_streamer.hpp"
#include "web_video_server/jpeg_streamers.hpp"
#include "web_video_server/png_streamers.hpp"
#include "web_video_server/vp8_streamer.hpp"
#include "web_video_server/h264_streamer.hpp"
#include "web_video_server/vp9_streamer.hpp"
#include "async_web_server_cpp/http_reply.hpp"

using namespace std::chrono_literals;
using namespace boost::placeholders;  // NOLINT

namespace web_video_server
{

WebVideoServer::WebVideoServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("web_video_server", options), handler_group_(
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found))
{
  declare_parameter("port", 8080);
  declare_parameter("verbose", true);
  declare_parameter("address", "0.0.0.0");
  declare_parameter("server_threads", 1);
  declare_parameter("publish_rate", -1.0);
  declare_parameter("default_stream_type", "mjpeg");

  get_parameter("port", port_);
  get_parameter("verbose", verbose_);
  get_parameter("address", address_);
  int server_threads;
  get_parameter("server_threads", server_threads);
  get_parameter("publish_rate", publish_rate_);
  get_parameter("default_stream_type", default_stream_type_);

  stream_types_["mjpeg"] = std::make_shared<MjpegStreamerType>();
  stream_types_["png"] = std::make_shared<PngStreamerType>();
  stream_types_["ros_compressed"] = std::make_shared<RosCompressedStreamerType>();
  stream_types_["vp8"] = std::make_shared<Vp8StreamerType>();
  stream_types_["h264"] = std::make_shared<H264StreamerType>();
  stream_types_["vp9"] = std::make_shared<Vp9StreamerType>();

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
    create_wall_timer(1s / publish_rate_, [this]() {restreamFrames(1s / publish_rate_);});
  }

  cleanup_timer_ = create_wall_timer(500ms, [this]() {cleanup_inactive_streams();});

  server_->run();
}

WebVideoServer::~WebVideoServer()
{
  server_->stop();
}

void WebVideoServer::restreamFrames(std::chrono::duration<double> max_age)
{
  std::scoped_lock lock(subscriber_mutex_);

  for (auto & subscriber : image_subscribers_) {
    subscriber->restreamFrame(max_age);
  }
}

void WebVideoServer::cleanup_inactive_streams()
{
  std::unique_lock lock(subscriber_mutex_, std::try_to_lock);
  if (lock) {
    auto new_end = std::partition(
      image_subscribers_.begin(), image_subscribers_.end(),
      [](const std::shared_ptr<ImageStreamer> & streamer) {return !streamer->isInactive();});
    if (verbose_) {
      for (auto itr = new_end; itr < image_subscribers_.end(); ++itr) {
        RCLCPP_INFO(get_logger(), "Removed Stream: %s", (*itr)->getTopic().c_str());
      }
    }
    image_subscribers_.erase(new_end, image_subscribers_.end());
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
  if (stream_types_.find(type) != stream_types_.end()) {
    std::string topic = request.get_query_param_value_or_default("topic", "");
    // Fallback for topics without corresponding compressed topics
    if (type == std::string("ros_compressed")) {
      std::string compressed_topic_name = topic + "/compressed";
      auto tnat = get_topic_names_and_types();
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
          get_logger(),
          "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
        type = "mjpeg";
      }
    }
    std::shared_ptr<ImageStreamer> streamer = stream_types_[type]->create_streamer(
      request, connection, shared_from_this());
    streamer->start();
    std::scoped_lock lock(subscriber_mutex_);
    image_subscribers_.push_back(streamer);
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
  std::shared_ptr<ImageStreamer> streamer = std::make_shared<JpegSnapshotStreamer>(
    request, connection, shared_from_this());
  streamer->start();

  std::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(streamer);
  return true;
}

bool WebVideoServer::handle_stream_viewer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, const char * begin,
  const char * end)
{
  std::string type = request.get_query_param_value_or_default("type", default_stream_type_);
  if (stream_types_.find(type) != stream_types_.end()) {
    std::string topic = request.get_query_param_value_or_default("topic", "");
    // Fallback for topics without corresponding compressed topics
    if (type == std::string("ros_compressed")) {
      std::string compressed_topic_name = topic + "/compressed";
      auto tnat = get_topic_names_and_types();
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
          get_logger(),
          "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
        type = "mjpeg";
      }
    }

    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Content-type", "text/html;")
    .write(connection);

    std::stringstream ss;
    ss << "<html><head><title>" << topic << "</title></head><body>";
    ss << "<h1>" << topic << "</h1>";
    ss << stream_types_[type]->create_viewer(request);
    ss << "</body></html>";
    connection->write(ss.str());
  } else {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(
      request, connection, begin, end);
  }
  return true;
}

bool WebVideoServer::handle_list_streams(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, const char * begin,
  const char * end)
{
  std::vector<std::string> image_topics;
  std::vector<std::string> camera_info_topics;
  auto tnat = get_topic_names_and_types();
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // skip over topics with more than one type
      continue;
    }
    auto & topic_name = topic_and_types.first;
    auto & topic_type = topic_and_types.second[0];  // explicitly take the first

    if (topic_type == "sensor_msgs/msg/Image") {
      image_topics.push_back(topic_name);
    } else if (topic_type == "sensor_msgs/msg/CameraInfo") {
      camera_info_topics.push_back(topic_name);
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
    "<head><title>ROS Image Topic List</title></head>"
    "<body><h1>Available ROS Image Topics:</h1>");
  connection->write("<ul>");
  for (std::string & camera_info_topic : camera_info_topics) {
    if (boost::algorithm::ends_with(camera_info_topic, "/camera_info")) {
      std::string base_topic = camera_info_topic.substr(
        0,
        camera_info_topic.size() - strlen("camera_info"));
      connection->write("<li>");
      connection->write(base_topic);
      connection->write("<ul>");
      std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
      for (; image_topic_itr != image_topics.end(); ) {
        if (boost::starts_with(*image_topic_itr, base_topic)) {
          connection->write("<li><a href=\"/stream_viewer?topic=");
          connection->write(*image_topic_itr);
          connection->write("\">");
          connection->write(image_topic_itr->substr(base_topic.size()));
          connection->write("</a> (");
          connection->write("<a href=\"/stream?topic=");
          connection->write(*image_topic_itr);
          connection->write("\">Stream</a>) (");
          connection->write("<a href=\"/snapshot?topic=");
          connection->write(*image_topic_itr);
          connection->write("\">Snapshot</a>)");
          connection->write("</li>");

          image_topic_itr = image_topics.erase(image_topic_itr);
        } else {
          ++image_topic_itr;
        }
      }
      connection->write("</ul>");
    }
    connection->write("</li>");
  }
  connection->write("</ul>");
  // Add the rest of the image topics that don't have camera_info.
  connection->write("<ul>");
  std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
  for (; image_topic_itr != image_topics.end(); ) {
    connection->write("<li><a href=\"/stream_viewer?topic=");
    connection->write(*image_topic_itr);
    connection->write("\">");
    connection->write(*image_topic_itr);
    connection->write("</a> (");
    connection->write("<a href=\"/stream?topic=");
    connection->write(*image_topic_itr);
    connection->write("\">Stream</a>) (");
    connection->write("<a href=\"/snapshot?topic=");
    connection->write(*image_topic_itr);
    connection->write("\">Snapshot</a>)");
    connection->write("</li>");

    image_topic_itr = image_topics.erase(image_topic_itr);
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
