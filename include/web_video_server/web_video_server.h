#ifndef WEB_VIDEO_SERVER_H_
#define WEB_VIDEO_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_server.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"

namespace web_video_server
{

/**
 * @class WebVideoServer
 * @brief
 */
class WebVideoServer
{
public:
  /**
   * @brief  Constructor
   * @return
   */
  WebVideoServer(rclcpp::Node::SharedPtr &nh, rclcpp::Node::SharedPtr &private_nh);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~WebVideoServer();

  /**
   * @brief  Starts the server and spins
   */
  void spin();

  void setup_cleanup_inactive_streams();

  bool handle_stream(const async_web_server_cpp::HttpRequest &request,
                     async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

  bool handle_stream_viewer(const async_web_server_cpp::HttpRequest &request,
                            async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

  bool handle_snapshot(const async_web_server_cpp::HttpRequest &request,
                       async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

  bool handle_list_streams(const async_web_server_cpp::HttpRequest &request,
                           async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

private:
  void restreamFrames(double max_age);
  void cleanup_inactive_streams();

  rclcpp::Node::SharedPtr nh_;
  rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr cleanup_timer_;
  int ros_threads_;
  double publish_rate_;
  int port_;
  std::string address_;
  boost::shared_ptr<async_web_server_cpp::HttpServer> server_;
  async_web_server_cpp::HttpRequestHandlerGroup handler_group_;

  std::vector<boost::shared_ptr<ImageStreamer> > image_subscribers_;
  std::map<std::string, boost::shared_ptr<ImageStreamerType> > stream_types_;
  boost::mutex subscriber_mutex_;
};

}

#endif
