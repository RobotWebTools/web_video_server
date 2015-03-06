#ifndef IMAGE_STREAMER_H_
#define IMAGE_STREAMER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "async_web_server_cpp/http_server.hpp"
#include "async_web_server_cpp/http_request.hpp"

namespace web_video_server
{

class ImageStreamer
{
public:
  ImageStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
                image_transport::ImageTransport it);

  virtual ~ImageStreamer();

  void start();

  bool isInactive();

  std::string getTopic()
  {
    return topic_;
  }

  /**
   * Restreams the last received image frame if older than max_age.
   */
  virtual void restreamFrame(double max_age);
protected:
  virtual void sendImage(const cv::Mat &, const ros::WallTime &time) = 0;

  virtual void initialize(const cv::Mat &);

  async_web_server_cpp::HttpConnectionPtr connection_;
  async_web_server_cpp::HttpRequest request_;
  bool inactive_;
  image_transport::Subscriber image_sub_;
  std::string topic_;
  int output_width_;
  int output_height_;
  bool invert_;
  ros::WallTime last_frame;
  cv::Mat output_size_image;
  boost::mutex send_mutex_;

private:
  image_transport::ImageTransport it_;
  bool initialized_;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};

class ImageStreamerType
{
public:
  virtual boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                           async_web_server_cpp::HttpConnectionPtr connection,
                                                           image_transport::ImageTransport it) = 0;

  virtual std::string create_viewer(const async_web_server_cpp::HttpRequest &request) = 0;
};

}

#endif
