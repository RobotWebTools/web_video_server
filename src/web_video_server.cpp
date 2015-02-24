#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "web_video_server/web_video_server.h"
#include "web_video_server/jpeg_streamers.h"
#include "web_video_server/vp8_streamer.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

struct mime_map
{
  const char* extension;
  const char* mime_type;
} mime_mapping[] =
  {
    { "gif", "image/gif" },
    { "htm", "text/html" },
    { "html", "text/html" },
    { "jpg", "image/jpeg" },
    { "png", "image/png" },
    { "css", "text/css" },
    { "js", "text/javascript" },
    { 0, 0 } // Marks end of list.
  };

std::string mimeExtensionToType(const std::string& extension)
{
  for (mime_map* m = mime_mapping; m->extension; ++m)
    {
      if (m->extension == extension)
	{
	  return m->mime_type;
	}
    }
  return "text/plain";
}

static void ros_connection_logger(async_web_server_cpp::HttpServerRequestHandler forward,
                                  const async_web_server_cpp::HttpRequest &request,
                                  async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                  const char* end)
{
  ROS_INFO_STREAM("Handling Request: " << request.uri);
  try
  {
    forward(request, connection, begin, end);
  }
  catch (std::exception &e)
  {
    ROS_WARN_STREAM("Error Handling Request: " << e.what());
  }
}

WebVideoServer::WebVideoServer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) :
    nh_(nh), image_transport_(nh), handler_group_(
        async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found))
{
  cleanup_timer_ = nh.createTimer(ros::Duration(0.5), boost::bind(&WebVideoServer::cleanup_inactive_streams, this));

  private_nh.param("port", port_, 8080);

  private_nh.param<std::string>("address", address_, "0.0.0.0");

  std::string wwwroot_;
  private_nh.param<std::string>("wwwroot", wwwroot_, ".");

  bool www_file_server_;
  private_nh.param<bool>("www_file_server", www_file_server_, false);

  int server_threads;
  private_nh.param("server_threads", server_threads, 1);

  private_nh.param("ros_threads", ros_threads_, 2);

  stream_types_["mjpeg"] = boost::shared_ptr<ImageStreamerType>(new MjpegStreamerType());
  stream_types_["vp8"] = boost::shared_ptr<ImageStreamerType>(new Vp8StreamerType());

  handler_group_.addHandlerForPath("/stream", boost::bind(&WebVideoServer::handle_stream, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/stream_viewer",
                                   boost::bind(&WebVideoServer::handle_stream_viewer, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/snapshot", boost::bind(&WebVideoServer::handle_snapshot, this, _1, _2, _3, _4));

  if ( www_file_server_ == true ) {
    handler_group_.addHandlerForPath("/.*", boost::bind(&WebVideoServer::handle_www_root, this, wwwroot_,  _1, _2, _3, _4));
  }
  else {
    handler_group_.addHandlerForPath("/", boost::bind(&WebVideoServer::handle_list_streams, this, _1, _2, _3, _4));
  }

  server_.reset(
      new async_web_server_cpp::HttpServer(address_, boost::lexical_cast<std::string>(port_),
                                           boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4),
                                           server_threads));
}

WebVideoServer::~WebVideoServer()
{
}

void WebVideoServer::spin()
{
  server_->run();
  ROS_INFO_STREAM("Waiting For connections on " << address_ << ":" << port_);
  ros::MultiThreadedSpinner spinner(ros_threads_);
  spinner.spin();
  server_->stop();
}

void WebVideoServer::cleanup_inactive_streams()
{
  boost::mutex::scoped_lock lock(subscriber_mutex_, boost::try_to_lock);
  if (lock)
  {
    typedef std::vector<boost::shared_ptr<ImageStreamer> >::iterator itr_type;
    itr_type new_end = std::remove_if(image_subscribers_.begin(), image_subscribers_.end(),
                                      boost::bind(&ImageStreamer::isInactive, _1));
    for (itr_type itr = new_end; itr < image_subscribers_.end(); ++itr)
    {
      ROS_INFO_STREAM("Removed Stream: " << (*itr)->getTopic());
    }
    image_subscribers_.erase(new_end, image_subscribers_.end());
  }
}

void WebVideoServer::handle_stream(const async_web_server_cpp::HttpRequest &request,
                                   async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                   const char* end)
{
  std::string type = request.get_query_param_value_or_default("type", "mjpeg");
  if (stream_types_.find(type) != stream_types_.end())
  {
    boost::shared_ptr<ImageStreamer> streamer = stream_types_[type]->create_streamer(request, connection,
                                                                                     image_transport_);
    streamer->start();
    boost::mutex::scoped_lock lock(subscriber_mutex_);
    image_subscribers_.push_back(streamer);
  }
  else
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin,
                                                                                             end);
  }
}

void WebVideoServer::handle_snapshot(const async_web_server_cpp::HttpRequest &request,
                                     async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                     const char* end)
{
  boost::shared_ptr<ImageStreamer> streamer(new JpegSnapshotStreamer(request, connection, image_transport_));
  streamer->start();

  boost::mutex::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(streamer);
}

void WebVideoServer::handle_stream_viewer(const async_web_server_cpp::HttpRequest &request,
                                          async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                          const char* end)
{
  std::string type = request.get_query_param_value_or_default("type", "mjpeg");
  if (stream_types_.find(type) != stream_types_.end())
  {
    std::string topic = request.get_query_param_value_or_default("topic", "");

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
}

void WebVideoServer::handle_www_root(std::string wwwroot, const async_web_server_cpp::HttpRequest &request,
                                         async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                         const char* end)
{
  std::string request_path = request.path;
  if (request_path == "/")
    request_path = "/index.html";

  std::size_t last_slash_pos = request_path.find_last_of("/");
  std::size_t last_dot_pos = request_path.find_last_of(".");
  std::string extension;
  if (last_dot_pos != std::string::npos && last_dot_pos > last_slash_pos)
    {
      extension = request_path.substr(last_dot_pos + 1);
    }

  // Open the file to send back.
  std::string full_path = wwwroot + request_path;
  std::ifstream is(full_path.c_str(), std::ios::in | std::ios::binary);
  if (!is)
    {
      // not found reply.
      async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin,
											       end);
      ROS_INFO("Http request from client: %s - file not found", full_path.c_str());
      return;
    }

    // get length of file:
    is.seekg (0, is.end);
    int length = is.tellg();
    is.seekg (0, is.beg);

    char * buffer = new char [length];

    // read data as a block:
    is.read (buffer,length);

    if (!is)
    {
      // not found reply.
      async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin,
											       end);
      ROS_INFO("Http request from client: %s - file found but incomplete read.", full_path.c_str());
      return;
    }

    async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close")
      .header("Server", "web_video_server")
      .header("Content-Length", boost::lexical_cast < std::string > ( is.gcount() ) )
      .header("Content-type", mimeExtensionToType(extension) ).write(connection);

    is.close();

    // NOTE: this must be doable more efficiently ! How to pass buffer directly without copying it into a string ?
    connection->write( std::string(buffer, length ) );

    delete[] buffer;
}

void WebVideoServer::handle_list_streams(const async_web_server_cpp::HttpRequest &request,
                                         async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                         const char* end)
{
  std::string image_message_type = ros::message_traits::datatype<sensor_msgs::Image>();
  std::string camera_info_message_type = ros::message_traits::datatype<sensor_msgs::CameraInfo>();

  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it;
  std::vector<std::string> image_topics;
  std::vector<std::string> camera_info_topics;
  for (it = topics.begin(); it != topics.end(); ++it)
  {
    const ros::master::TopicInfo &topic = *it;
    if (topic.datatype == image_message_type)
    {
      image_topics.push_back(topic.name);
    }
    else if (topic.datatype == camera_info_message_type)
    {
      camera_info_topics.push_back(topic.name);
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
  connection->write("</ul></body></html>");
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "web_video_server");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  web_video_server::WebVideoServer server(nh, private_nh);
  server.spin();

  return (0);
}
