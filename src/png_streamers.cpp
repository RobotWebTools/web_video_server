#include "web_video_server/png_streamers.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

PngStreamer::PngStreamer(const async_web_server_cpp::HttpRequest &request,
                         async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageTransportImageStreamer(request, connection, nh), stream_(connection)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 3);
  stream_.sendInitialHeader();
}

PngStreamer::~PngStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void PngStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
#if CV_VERSION_MAJOR >= 3
  encode_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
#else
  encode_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
#endif
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".png", img, encoded_buffer, encode_params);

  stream_.sendPartAndClear(time, "image/png", encoded_buffer);
}

boost::shared_ptr<ImageStreamer> PngStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                                  async_web_server_cpp::HttpConnectionPtr connection,
                                                                  ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new PngStreamer(request, connection, nh));
}

std::string PngStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}

PngSnapshotStreamer::PngSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                                         async_web_server_cpp::HttpConnectionPtr connection,
                                         ros::NodeHandle& nh) :
    ImageTransportImageStreamer(request, connection, nh)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 3);
}

PngSnapshotStreamer::~PngSnapshotStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void PngSnapshotStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
#if CV_VERSION_MAJOR >= 3
  encode_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
#else
  encode_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
#endif
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".png", img, encoded_buffer, encode_params);

  char stamp[20];
  sprintf(stamp, "%.06lf", time.toSec());
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
      .header("Connection", "close")
      .header("Server", "web_video_server")
      .header("Cache-Control",
              "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, "
              "max-age=0")
      .header("X-Timestamp", stamp)
      .header("Pragma", "no-cache")
      .header("Content-type", "image/png")
      .header("Access-Control-Allow-Origin", "*")
      .header("Content-Length",
              boost::lexical_cast<std::string>(encoded_buffer.size()))
      .write(connection_);
  connection_->write_and_clear(encoded_buffer);
  inactive_ = true;
}

}
