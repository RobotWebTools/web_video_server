#ifndef JPEG_STREAMERS_H_
#define JPEG_STREAMERS_H_

#include <image_transport/image_transport.h>
#include "web_video_server/image_streamer.h"
#include "web_video_server/http_server/http_request.hpp"
#include "web_video_server/http_server/http_connection.hpp"

namespace web_video_server
{

class MjpegStreamer : public ImageStreamer
{
public:
  MjpegStreamer(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection,
      image_transport::ImageTransport it);

protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time);

private:
  int quality_;
};

class MjpegStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection,
      image_transport::ImageTransport it);

  std::string create_viewer(const http_server::HttpRequest &request);
};


class JpegSnapshotStreamer : public ImageStreamer
{
public:
  JpegSnapshotStreamer(const http_server::HttpRequest &request,
      http_server::HttpConnectionPtr connection,
      image_transport::ImageTransport it);

protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time);

private:
  int quality_;
};

}

#endif
