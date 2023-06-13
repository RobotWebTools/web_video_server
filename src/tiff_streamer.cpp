#include "web_video_server/tiff_streamer.h"
#include "async_web_server_cpp/http_reply.hpp"
#define MB 1024*1024
namespace web_video_server
{

// TiffStreamer::TiffStreamer(const async_web_server_cpp::HttpRequest &request,
//                          async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
//   ImageTransportImageStreamer(request, connection, nh), stream_(connection)
// {
//   quality_ = request.get_query_param_value_or_default<int>("quality", 3);
//   stream_.sendInitialHeader();
// }

// TiffStreamer::~TiffStreamer()
// {
//   this->inactive_ = true;
//   boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
// }

// void TiffStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
// {
//   std::vector<int> encode_params;
// #if CV_VERSION_MAJOR >= 3
//   encode_params.push_back(cv::IMWRITE_Tiff_COMPRESSION);
// #else
//   encode_params.push_back(CV_IMWRITE_Tiff_COMPRESSION);
// #endif
//   encode_params.push_back(quality_);

//   std::vector<uchar> encoded_buffer;
//   cv::imencode(".Tiff", img, encoded_buffer, encode_params);

//   stream_.sendPartAndClear(time, "image/Tiff", encoded_buffer);
// }

// boost::shared_ptr<ImageStreamer> TiffStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
//                                                                   async_web_server_cpp::HttpConnectionPtr connection,
//                                                                   ros::NodeHandle& nh)
// {
//   return boost::shared_ptr<ImageStreamer>(new TiffStreamer(request, connection, nh));
// }

// std::string TiffStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
// {
//   std::stringstream ss;
//   ss << "<img src=\"/stream?";
//   ss << request.query;
//   ss << "\"></img>";
//   return ss.str();
// }

TiffSnapshotStreamer::TiffSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                                         async_web_server_cpp::HttpConnectionPtr connection,
                                         ros::NodeHandle& nh) :
    ImageTransportImageStreamer(request, connection, nh)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 1);
}

TiffSnapshotStreamer::~TiffSnapshotStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void TiffSnapshotStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
#if CV_VERSION_MAJOR >= 3
  encode_params.push_back(cv::IMWRITE_TIFF_COMPRESSION);
#else
  encode_params.push_back(CV_IMWRITE_TIFF_COMPRESSION);
#endif
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  encoded_buffer.resize(2*MB);

  if(img.empty()){
    ROS_INFO("Image is empty?");
    return;
  }
  ROS_INFO("Tag1");
  try{
    cv::imencode(".png", img, encoded_buffer, encode_params);
  }
  catch(const std::exception &e){
    ROS_INFO_STREAM("Error: " << e.what());
    return;
  }
  
  ROS_INFO_STREAM("buffer size: " << encoded_buffer.size());

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
      .header("Content-type", "image/tiff")
      .header("Access-Control-Allow-Origin", "*")
      .header("Content-Length",
              boost::lexical_cast<std::string>(encoded_buffer.size()))
      .write(connection_);
  connection_->write_and_clear(encoded_buffer);
  inactive_ = true;
}

boost::shared_ptr<ImageStreamer> TiffSnapshotType::create_snapshot(const async_web_server_cpp::HttpRequest &request,
                                         async_web_server_cpp::HttpConnectionPtr connection,
                                         ros::NodeHandle& nh){
  return boost::shared_ptr<ImageStreamer>(new TiffSnapshotStreamer(request, connection, nh));
}
}
