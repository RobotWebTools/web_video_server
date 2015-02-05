#ifndef JPEG_STREAMERS_H_
#define JPEG_STREAMERS_H_

#include <image_transport/image_transport.h>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"

namespace web_video_server
{

class MjpegStreamer : public ImageStreamer
{
public:
  MjpegStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
                image_transport::ImageTransport it);

protected:
  virtual void sendImage(const cv::Mat &, const ros::WallTime &time);

private:
  int quality_;
};

class MjpegStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                   async_web_server_cpp::HttpConnectionPtr connection,
                                                   image_transport::ImageTransport it);

  std::string create_viewer(const async_web_server_cpp::HttpRequest &request);
};

class JpegSnapshotStreamer : public ImageStreamer
{
public:
  JpegSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                       async_web_server_cpp::HttpConnectionPtr connection, image_transport::ImageTransport it);

protected:
  virtual void sendImage(const cv::Mat &, const ros::WallTime &time);

private:
  int quality_;
};

}

#endif
