#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <chrono>
#include <vector>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

#include "web_video_server/web_video_server.h"
#include "web_video_server/ros_compressed_streamer.h"
#include "web_video_server/jpeg_streamers.h"
#include "web_video_server/png_streamers.h"
#include "web_video_server/vp8_streamer.h"
#include "web_video_server/h264_streamer.h"
#include "web_video_server/vp9_streamer.h"
#include "async_web_server_cpp/http_reply.hpp"

using namespace std::chrono_literals;

namespace web_video_server
{

static bool __verbose;

static std::string __default_stream_type;

static bool ros_connection_logger(async_web_server_cpp::HttpServerRequestHandler forward,
                                  const async_web_server_cpp::HttpRequest &request,
                                  async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                  const char* end)
{
  if (__verbose)
  {
    // TODO reenable
    // RCLCPP_INFO(nh->get_logger(), "Handling Request: %s", request.uri.c_str());
  }
  try
  {
    forward(request, connection, begin, end);
    return true;
  }
  catch (std::exception &e)
  {
    // TODO reenable
    // RCLCPP_WARN(nh->get_logger(), "Error Handling Request: %s", e.what());
    return false;
  }
  return false;
}

WebVideoServer::WebVideoServer(rclcpp::Node::SharedPtr &nh, rclcpp::Node::SharedPtr &private_nh) :
    nh_(nh), handler_group_(
        async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found))
{
  rclcpp::Parameter parameter;
  if (private_nh->get_parameter("port", parameter)) {
    port_ = parameter.as_int();
  } else {
    port_ = 8080;
  }
  if (private_nh->get_parameter("verbose", parameter)) {
    __verbose = parameter.as_bool();
  } else {
    __verbose = true;
  }

  if (private_nh->get_parameter("address", parameter)) {
    address_ = parameter.as_string();
  } else {
    address_ = "0.0.0.0";
  }

  int server_threads;
  if (private_nh->get_parameter("server_threads", parameter)) {
    server_threads = parameter.as_int();
  } else {
    server_threads = 1;
  }

  if (private_nh->get_parameter("ros_threads", parameter)) {
    ros_threads_ = parameter.as_int();
  } else {
    ros_threads_ = 2;
  }
  if (private_nh->get_parameter("publish_rate", parameter)) {
    publish_rate_ = parameter.as_double();
  } else {
    publish_rate_ = -1.0;
  }

  if (private_nh->get_parameter("default_stream_type", parameter)) {
    __default_stream_type = parameter.as_string();
  } else {
    __default_stream_type = "mjpeg";
  }

  stream_types_["mjpeg"] = boost::shared_ptr<ImageStreamerType>(new MjpegStreamerType());
  stream_types_["png"] = boost::shared_ptr<ImageStreamerType>(new PngStreamerType());
  stream_types_["ros_compressed"] = boost::shared_ptr<ImageStreamerType>(new RosCompressedStreamerType());
  stream_types_["vp8"] = boost::shared_ptr<ImageStreamerType>(new Vp8StreamerType());
  stream_types_["h264"] = boost::shared_ptr<ImageStreamerType>(new H264StreamerType());
  stream_types_["vp9"] = boost::shared_ptr<ImageStreamerType>(new Vp9StreamerType());

  handler_group_.addHandlerForPath("/", boost::bind(&WebVideoServer::handle_list_streams, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/stream", boost::bind(&WebVideoServer::handle_stream, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/stream_viewer",
                                   boost::bind(&WebVideoServer::handle_stream_viewer, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/snapshot", boost::bind(&WebVideoServer::handle_snapshot, this, _1, _2, _3, _4));

  try
  {
    server_.reset(
        new async_web_server_cpp::HttpServer(address_, boost::lexical_cast<std::string>(port_),
                                             boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4),
                                             server_threads));
  }
  catch(boost::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Exception when creating the web server! %s:%d", address_.c_str(), port_);
    throw;
  }
}

WebVideoServer::~WebVideoServer()
{
}

void WebVideoServer::setup_cleanup_inactive_streams()
{
  std::function<void()> callback = std::bind(&WebVideoServer::cleanup_inactive_streams, this);
  cleanup_timer_ = nh_->create_wall_timer(500ms, callback);
}

void WebVideoServer::spin()
{
  server_->run();
  RCLCPP_INFO(nh_->get_logger(), "Waiting For connections on %s:%d", address_.c_str(), port_);
  rclcpp::executors::MultiThreadedExecutor spinner(rclcpp::ExecutorOptions(), ros_threads_);
  spinner.add_node(nh_);
  if ( publish_rate_ > 0 ) {
    nh_->create_wall_timer(1s / publish_rate_, [this](){restreamFrames(1.0 / publish_rate_);});
  }
  spinner.spin();
  server_->stop();
}

void WebVideoServer::restreamFrames( double max_age )
{
  boost::mutex::scoped_lock lock(subscriber_mutex_);

  typedef std::vector<boost::shared_ptr<ImageStreamer> >::iterator itr_type;

  for (itr_type itr = image_subscribers_.begin(); itr < image_subscribers_.end(); ++itr)
    {
      (*itr)->restreamFrame( max_age );
    }
}

void WebVideoServer::cleanup_inactive_streams()
{
  boost::mutex::scoped_lock lock(subscriber_mutex_, boost::try_to_lock);
  if (lock)
  {
    typedef std::vector<boost::shared_ptr<ImageStreamer> >::iterator itr_type;
    itr_type new_end = std::partition(image_subscribers_.begin(), image_subscribers_.end(),
                                      !boost::bind(&ImageStreamer::isInactive, _1));
    if (__verbose)
    {
      for (itr_type itr = new_end; itr < image_subscribers_.end(); ++itr)
      {
        RCLCPP_INFO(nh_->get_logger(), "Removed Stream: %s", (*itr)->getTopic().c_str());
      }
    }
    image_subscribers_.erase(new_end, image_subscribers_.end());
  }
}

bool WebVideoServer::handle_stream(const async_web_server_cpp::HttpRequest &request,
                                   async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                   const char* end)
{
  std::string type = request.get_query_param_value_or_default("type", __default_stream_type);
  if (stream_types_.find(type) != stream_types_.end())
  {
    std::string topic = request.get_query_param_value_or_default("topic", "");
    // Fallback for topics without corresponding compressed topics
    if (type == std::string("ros_compressed"))
    {
      std::string compressed_topic_name = topic + "/compressed";
      auto tnat = nh_->get_topic_names_and_types();
      bool did_find_compressed_topic = false;
      for (auto topic_and_types : tnat) {
        if (topic_and_types.second.size() > 1) {
          // explicitly avoid topics with more than one type
          break;
        }
        auto & topic_name = topic_and_types.first;
        if(topic_name == compressed_topic_name || (topic_name.find("/") == 0 && topic_name.substr(1) == compressed_topic_name)){
          did_find_compressed_topic = true;
          break;
        }
      }
      if (!did_find_compressed_topic)
      {
        RCLCPP_WARN(nh_->get_logger(), "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
        type = "mjpeg";
      }
    }
    boost::shared_ptr<ImageStreamer> streamer = stream_types_[type]->create_streamer(request, connection, nh_);
    streamer->start();
    boost::mutex::scoped_lock lock(subscriber_mutex_);
    image_subscribers_.push_back(streamer);
  }
  else
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin,
                                                                                             end);
  }
  return true;
}

bool WebVideoServer::handle_snapshot(const async_web_server_cpp::HttpRequest &request,
                                     async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                     const char* end)
{
  boost::shared_ptr<ImageStreamer> streamer(new JpegSnapshotStreamer(request, connection, nh_));
  streamer->start();

  boost::mutex::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(streamer);
  return true;
}

bool WebVideoServer::handle_stream_viewer(const async_web_server_cpp::HttpRequest &request,
                                          async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                          const char* end)
{
  std::string type = request.get_query_param_value_or_default("type", __default_stream_type);
  if (stream_types_.find(type) != stream_types_.end())
  {
    std::string topic = request.get_query_param_value_or_default("topic", "");
    // Fallback for topics without corresponding compressed topics
    if (type == std::string("ros_compressed"))
    {

      std::string compressed_topic_name = topic + "/compressed";
      auto tnat = nh_->get_topic_names_and_types();
      bool did_find_compressed_topic = false;
      for (auto topic_and_types : tnat) {
        if (topic_and_types.second.size() > 1) {
          // explicitly avoid topics with more than one type
          break;
        }
        auto & topic_name = topic_and_types.first;
        if(topic_name == compressed_topic_name || (topic_name.find("/") == 0 && topic_name.substr(1) == compressed_topic_name)){
          did_find_compressed_topic = true;
          break;
        }
      }
      if (!did_find_compressed_topic)
      {
        RCLCPP_WARN(nh_->get_logger(), "Could not find compressed image topic for %s, falling back to mjpeg", topic.c_str());
        type = "mjpeg";
      }
    }

    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
        "Server", "web_video_server").header("Content-type", "text/html;").write(connection);

    std::stringstream ss;
    ss << "<html><head><title>" << topic << "</title></head><body>";
    ss << "<h1>" << topic << "</h1>";
    ss << stream_types_[type]->create_viewer(request);
    ss << "</body></html>";
    connection->write(ss.str());
  }
  else
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin,
                                                                                             end);
  }
  return true;
}

bool WebVideoServer::handle_list_streams(const async_web_server_cpp::HttpRequest &request,
                                         async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                         const char* end)
{
  std::vector<std::string> image_topics;
  std::vector<std::string> camera_info_topics;
  auto tnat = nh_->get_topic_names_and_types();
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // explicitly avoid topics with more than one type
      break;
    }
    auto & topic_name = topic_and_types.first;
    auto & topic_type = topic_and_types.second[0];  // explicitly take the first
    // TODO debugging
    fprintf(stderr, "topic_type: %s\n", topic_type.c_str());
    if (topic_type == "sensor_msgs/msg/Image")
    {
      image_topics.push_back(topic_name);
    }
    else if (topic_type == "sensor_msgs/msg/CameraInfo")
    {
      camera_info_topics.push_back(topic_name);
    }
  }

  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Content-type", "text/html;").write(connection);

  connection->write("<html>"
                    "<head><title>ROS Image Topic List</title></head>"
                    "<body><h1>Available ROS Image Topics:</h1>");
  connection->write("<ul>");
  BOOST_FOREACH(std::string & camera_info_topic, camera_info_topics)
  {
    if (boost::algorithm::ends_with(camera_info_topic, "/camera_info"))
    {
      std::string base_topic = camera_info_topic.substr(0, camera_info_topic.size() - strlen("camera_info"));
      connection->write("<li>");
      connection->write(base_topic);
      connection->write("<ul>");
      std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
      for (; image_topic_itr != image_topics.end();)
      {
        if (boost::starts_with(*image_topic_itr, base_topic))
        {
          connection->write("<li><a href=\"/stream_viewer?topic=");
          connection->write(*image_topic_itr);
          connection->write("\">");
          connection->write(image_topic_itr->substr(base_topic.size()));
          connection->write("</a> (");
          connection->write("<a href=\"/snapshot?topic=");
          connection->write(*image_topic_itr);
          connection->write("\">Snapshot</a>)");
          connection->write("</li>");

          image_topic_itr = image_topics.erase(image_topic_itr);
        }
        else
        {
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
  for (; image_topic_itr != image_topics.end();) {
    connection->write("<li><a href=\"/stream_viewer?topic=");
    connection->write(*image_topic_itr);
    connection->write("\">");
    connection->write(*image_topic_itr);
    connection->write("</a> (");
    connection->write("<a href=\"/snapshot?topic=");
    connection->write(*image_topic_itr);
    connection->write("\">Snapshot</a>)");
    connection->write("</li>");

    image_topic_itr = image_topics.erase(image_topic_itr);
  }
  connection->write("</ul></body></html>");
  return true;
}

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("web_video_server");
  auto private_nh = std::make_shared<rclcpp::Node>("_web_video_server");

  web_video_server::WebVideoServer server(nh, private_nh);
  server.setup_cleanup_inactive_streams();
  server.spin();

  return (0);
}
