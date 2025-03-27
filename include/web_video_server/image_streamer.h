#ifndef IMAGE_STREAMER_H_
#define IMAGE_STREAMER_H_

#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_server.hpp"
#include "web_video_server/utils.h"
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <opencv2/opencv.hpp>
#include "web_video_server/utils.h"
#include "async_web_server_cpp/http_server.hpp"
#include "async_web_server_cpp/http_request.hpp"

namespace web_video_server
{

class ImageStreamer
{
public:
  ImageStreamer(const async_web_server_cpp::HttpRequest &request,
		async_web_server_cpp::HttpConnectionPtr connection,
		rclcpp::Node::SharedPtr nh);

  virtual void start() = 0;
  virtual ~ImageStreamer();

  bool isInactive()
  {
    return inactive_;
  }
  ;

  /**
   * Restreams the last received image frame if older than max_age.
   */
  virtual void restreamFrame(double max_age) = 0;

  std::string getTopic()
  {
    return topic_;
  }
  ;
protected:
  async_web_server_cpp::HttpConnectionPtr connection_;
  async_web_server_cpp::HttpRequest request_;
  rclcpp::Node::SharedPtr nh_;
  bool inactive_;
  image_transport::Subscriber image_sub_;
  std::string topic_;
};


class ImageTransportImageStreamer : public ImageStreamer
{
public:
  ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
			      rclcpp::Node::SharedPtr nh);
  virtual ~ImageTransportImageStreamer();

  virtual void start();

protected:
  virtual void sendImage(const cv::Mat &, const rclcpp::Time &time) = 0;
  virtual void restreamFrame(double max_age);
  virtual void initialize(const cv::Mat &);

  /**
   * @brief Detects QoS profile settings used by publishers on a specified topic.
   * 
   * This function queries the ROS2 middleware to discover publishers on the 
   * given topic and extracts their QoS profile settings. It's useful for 
   * creating subscribers that automatically match the publisher's QoS.
   * 
   * @param topic_name The full name of the topic to query.
   * @return std::optional<rmw_qos_profile_t> The detected QoS profile if a publisher
   *         is found, std::nullopt otherwise
   */
  std::optional<rmw_qos_profile_t> detect_publisher_qos(const std::string &topic_name);

  image_transport::Subscriber image_sub_;
  int output_width_;
  int output_height_;
  bool invert_;
  std::string default_transport_;
  std::string qos_profile_name_;

  rclcpp::Time last_frame;
  cv::Mat output_size_image;
  boost::mutex send_mutex_;

private:
  image_transport::ImageTransport it_;
  bool initialized_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
};

class ImageStreamerType
{
public:
  virtual boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                           async_web_server_cpp::HttpConnectionPtr connection,
                                                           rclcpp::Node::SharedPtr nh) = 0;

  virtual std::string create_viewer(const async_web_server_cpp::HttpRequest &request) = 0;
};

}

#endif
