#include "web_video_server/jpeg_streamers.h"
#include "web_video_server/http_server/http_reply.hpp"

namespace web_video_server
{

MjpegStreamer::MjpegStreamer(const http_server::HttpRequest &request,
    http_server::HttpConnectionPtr connection,
    image_transport::ImageTransport it)
    : ImageStreamer(request, connection, it)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);

  http_server::HttpReply::builder(http_server::HttpReply::ok)
      .header("Connection", "close")
      .header("Server", "web_video_server")
      .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
      .header("Pragma", "no-cache")
      .header("Content-type", "multipart/x-mixed-replace;boundary=--boundarydonotcross ")
      .header("Access-Control-Allow-Origin", "*")
      .write(connection);
  connection->write("--boundarydonotcross \r\n");
}

void MjpegStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  char stamp[20];
  sprintf(stamp, "%.06lf", time.toSec());
  boost::shared_ptr<std::vector<http_server::HttpHeader> > headers(new std::vector<http_server::HttpHeader>());
  headers->push_back(http_server::HttpHeader("Content-type", "image/jpeg"));
  headers->push_back(http_server::HttpHeader("X-Timestamp", stamp));
  headers->push_back(http_server::HttpHeader("Content-Length", boost::lexical_cast<std::string>(encoded_buffer.size())));
  connection_->write(http_server::HttpReply::to_buffers(*headers), headers);
  connection_->write_and_clear(encoded_buffer);
  connection_->write("\r\n--boundarydonotcross \r\n");
}

boost::shared_ptr<ImageStreamer> MjpegStreamerType::create_streamer(const http_server::HttpRequest &request,
    http_server::HttpConnectionPtr connection,
    image_transport::ImageTransport it)
{
  return boost::shared_ptr<ImageStreamer>(new MjpegStreamer(request, connection, it));
}

std::string MjpegStreamerType::create_viewer(const http_server::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}

JpegSnapshotStreamer::JpegSnapshotStreamer(const http_server::HttpRequest &request,
    http_server::HttpConnectionPtr connection,
    image_transport::ImageTransport it)
    : ImageStreamer(request, connection, it)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
}

void JpegSnapshotStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  char stamp[20];
  sprintf(stamp, "%.06lf", time.toSec());
  http_server::HttpReply::builder(http_server::HttpReply::ok)
      .header("Connection", "close")
      .header("Server", "web_video_server")
      .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
      .header("X-Timestamp", stamp)
      .header("Pragma", "no-cache")
      .header("Content-type", "image/jpeg")
      .header("Access-Control-Allow-Origin", "*")
      .header("Content-Length", boost::lexical_cast<std::string>(encoded_buffer.size()))
      .write(connection_);
  connection_->write_and_clear(encoded_buffer);
  inactive_ = true;
}

}
