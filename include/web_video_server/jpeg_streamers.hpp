#ifndef JPEG_STREAMERS_H_
#define JPEG_STREAMERS_H_

#include <image_transport/image_transport.hpp>
#include "web_video_server/image_streamer.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.hpp"

namespace web_video_server
{

class MjpegStreamer : public ImageTransportImageStreamer
{
public:
  MjpegStreamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node);
  ~MjpegStreamer();

protected:
  virtual void sendImage(const cv::Mat &, const rclcpp::Time & time);

private:
  MultipartStream stream_;
  int quality_;
};

class MjpegStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node);
  std::string create_viewer(const async_web_server_cpp::HttpRequest & request);
};

class JpegSnapshotStreamer : public ImageTransportImageStreamer
{
public:
  JpegSnapshotStreamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node);
  ~JpegSnapshotStreamer();

protected:
  virtual void sendImage(const cv::Mat &, const rclcpp::Time & time);

private:
  int quality_;
};

}

#endif
