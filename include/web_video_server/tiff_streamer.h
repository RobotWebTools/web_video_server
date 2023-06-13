#ifndef TIFF_STREAMERS_H_
#define TIFF_STREAMERS_H_

#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.h"

namespace web_video_server
{

// class TiffStreamer : public ImageTransportImageStreamer
// {
// public:
//   TiffStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
//               ros::NodeHandle& nh);
//   ~TiffStreamer();
// protected:
//   virtual void sendImage(const cv::Mat &, const ros::Time &time);

// private:
//   MultipartStream stream_;
//   int quality_;
// };

// class TiffStreamerType : public ImageStreamerType
// {
// public:
//   boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
//                                                    async_web_server_cpp::HttpConnectionPtr connection,
//                                                    ros::NodeHandle& nh);
//   std::string create_viewer(const async_web_server_cpp::HttpRequest &request);
// };

class TiffSnapshotStreamer : public ImageTransportImageStreamer
{
public:
  TiffSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                      async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh);
  ~TiffSnapshotStreamer();
protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time);

private:
  int quality_;
};

class TiffSnapshotType : public ImageSnapshotType{
public:
  boost::shared_ptr<ImageStreamer> create_snapshot(const async_web_server_cpp::HttpRequest &request,
                                                   async_web_server_cpp::HttpConnectionPtr connection,
                                                   ros::NodeHandle& nh);
};
}

#endif
