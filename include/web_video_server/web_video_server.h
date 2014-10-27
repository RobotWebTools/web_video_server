#ifndef WEB_VIDEO_SERVER_H_
#define WEB_VIDEO_SERVER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "web_video_server/image_streamer.h"
#include "web_video_server/http_server/http_server.hpp"
#include "web_video_server/http_server/http_request.hpp"
#include "web_video_server/http_server/http_connection.hpp"

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
  WebVideoServer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

  /**
  * @brief  Destructor - Cleans up
  */
  virtual ~WebVideoServer();


  /**
  * @brief  Starts the server and spins
  */
  void spin();

  void handle_stream(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection);

  void handle_stream_viewer(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection);

  void handle_snapshot(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection);

  void handle_list_streams(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection);


private:
  void cleanup_inactive_streams();

  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  ros::Timer cleanup_timer_;
  int ros_threads_;
  boost::shared_ptr<http_server::HttpServer> server_;
  web_video_server::http_server::RequestHandlerGroup handler_group_;

  std::vector<boost::shared_ptr<ImageStreamer> > image_subscribers_;
  std::map<std::string, boost::shared_ptr<ImageStreamerType> > stream_types_;
  boost::mutex subscriber_mutex_;
};

}

#endif
