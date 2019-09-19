#ifndef ROS_COMPRESSED_STREAMERS_H_
#define ROS_COMPRESSED_STREAMERS_H_

#include <sensor_msgs/msg/compressed_image.hpp>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.h"

namespace web_video_server
{

class RosCompressedStreamer : public ImageStreamer
{
public:
  RosCompressedStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
			rclcpp::Node::SharedPtr nh);
  ~RosCompressedStreamer();
  virtual void start();
  virtual void restreamFrame(double max_age);

protected:
  virtual void sendImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, const rclcpp::Time &time);

private:
  void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  MultipartStream stream_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Time last_frame;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr last_msg;
  boost::mutex send_mutex_;
};

class RosCompressedStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                   async_web_server_cpp::HttpConnectionPtr connection,
                                                   rclcpp::Node::SharedPtr nh);
  std::string create_viewer(const async_web_server_cpp::HttpRequest &request);
};

}

#endif
