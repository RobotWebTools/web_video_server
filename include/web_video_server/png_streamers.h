#ifndef PNG_STREAMERS_H_
#define PNG_STREAMERS_H_

#include <image_transport/image_transport.hpp>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.h"

namespace web_video_server
{

class PngStreamer : public ImageTransportImageStreamer
{
public:
  PngStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
              rclcpp::Node::SharedPtr nh);
  ~PngStreamer();
protected:
  virtual void sendImage(const cv::Mat &, const rclcpp::Time &time);

private:
  MultipartStream stream_;
  int quality_;
};

class PngStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                   async_web_server_cpp::HttpConnectionPtr connection,
                                                   rclcpp::Node::SharedPtr nh);
  std::string create_viewer(const async_web_server_cpp::HttpRequest &request);
};

class PngSnapshotStreamer : public ImageTransportImageStreamer
{
public:
  PngSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                      async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh);
  ~PngSnapshotStreamer();
protected:
  virtual void sendImage(const cv::Mat &, const rclcpp::Time &time);

private:
  int quality_;
};

}

#endif
