#ifndef IMAGE_STREAMER_H_
#define IMAGE_STREAMER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "web_video_server/http_server/http_server.hpp"
#include "web_video_server/http_server/http_request.hpp"

namespace web_video_server
{

class ImageStreamer
{
public:
  ImageStreamer(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection,
      image_transport::ImageTransport it);

  void start();

  bool isInactive();

  std::string getTopic()
  {
    return topic_;
  };
protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time) = 0;

  virtual void initialize(const cv::Mat &);

  http_server::HttpConnectionPtr connection_;
  http_server::HttpRequest request_;
  bool inactive_;
  image_transport::Subscriber image_sub_;
  std::string topic_;
  int output_width_;
  int output_height_;
  bool invert_;
private:
  image_transport::ImageTransport it_;
  bool initialized_;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};

class ImageStreamerType
{
public:
  virtual boost::shared_ptr<ImageStreamer> create_streamer(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection,
      image_transport::ImageTransport it) = 0;

  virtual std::string create_viewer(const http_server::HttpRequest &request) = 0;
};

}

#endif
