#include "web_video_server/ros_compressed_streamer.h"

#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace web_video_server
{

const int PARAM_NO_CHANGE_INT = -1;

RosCompressedStreamer::RosCompressedStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageStreamer(request, connection, nh), stream_(connection)
{
  jpeg_quality_ = request.get_query_param_value_or_default<int>("quality", PARAM_NO_CHANGE_INT);
  png_level_ = request.get_query_param_value_or_default<int>("level", PARAM_NO_CHANGE_INT);
  format_ = request.get_query_param_value_or_default<std::string>("format", "");

  stream_.sendInitialHeader();
}

RosCompressedStreamer::~RosCompressedStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void RosCompressedStreamer::start() {
  std::string compressed_topic = topic_ + "/compressed";
  updateImageParameters();
  image_sub_ = nh_.subscribe(compressed_topic, 1, &RosCompressedStreamer::imageCallback, this);
}

void RosCompressedStreamer::restreamFrame(double max_age)
{
  if (inactive_ || (last_msg == 0))
    return;

  if ( last_frame + ros::Duration(max_age) < ros::Time::now() ) {
    boost::mutex::scoped_lock lock(send_mutex_);
    sendImage(last_msg, ros::Time::now() ); // don't update last_frame, it may remain an old value.
  }
}

void RosCompressedStreamer::sendImage(const sensor_msgs::CompressedImageConstPtr &msg,
                                      const ros::Time &time) {
  try {
    std::string content_type;
    if(msg->format.find("jpeg") != std::string::npos || msg->format.find("jpg") != std::string::npos) {
      content_type = "image/jpeg";
    }
    else if(msg->format.find("png") != std::string::npos) {
      content_type = "image/png";
    }
    else {
      ROS_WARN_STREAM("Unknown ROS compressed image format: " << msg->format);
      return;
    }

    stream_.sendPart(time, content_type, boost::asio::buffer(msg->data), msg);
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    ROS_DEBUG("system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    ROS_ERROR_THROTTLE(30, "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    ROS_ERROR_THROTTLE(30, "exception");
    inactive_ = true;
    return;
  }
}


void RosCompressedStreamer::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg) {
  boost::mutex::scoped_lock lock(send_mutex_); // protects last_msg and last_frame
  last_msg = msg;
  last_frame = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
  sendImage(last_msg, last_frame);
}


void RosCompressedStreamer::updateImageParameters()
{
  dynamic_reconfigure::Config reconf_config;
  if (jpeg_quality_ > 0 && jpeg_quality_ <= 100) {
    dynamic_reconfigure::IntParameter param;
    param.name = "jpeg_quality";
    param.value = jpeg_quality_;
    ROS_DEBUG("setting jpeg quality to %d for topic %s", jpeg_quality_, topic_.c_str());
    reconf_config.ints.push_back(param);
  }
  if (png_level_ > 0 && png_level_ <= 9) {
    dynamic_reconfigure::IntParameter param;
    param.name = "png_level";
    param.value = png_level_;
    ROS_DEBUG("setting png compression level to %d for topic %s", png_level_, topic_.c_str());
    reconf_config.ints.push_back(param);
  }
  if (format_ == "jpeg" || format_ == "png") { // Only allow jpeg/png compression
    dynamic_reconfigure::StrParameter param;
    param.name = "format";
    param.value = format_;
    ROS_DEBUG("setting compression format to %s for topic %s", format_.c_str(), topic_.c_str());
    reconf_config.strs.push_back(param);
  }
  if (reconf_config.ints.size() > 0 || reconf_config.strs.size() > 0)
  {
    dynamic_reconfigure::ReconfigureRequest reconf_request;
    dynamic_reconfigure::ReconfigureResponse reconf_response;
    reconf_request.config = reconf_config;
    ROS_DEBUG("applying settings for %s", format_.c_str());
    ros::service::call(topic_ + "/compressed/set_parameters", reconf_request, reconf_response);
  }
}

boost::shared_ptr<ImageStreamer> RosCompressedStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
										 async_web_server_cpp::HttpConnectionPtr connection,
										 ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new RosCompressedStreamer(request, connection, nh));
}

std::string RosCompressedStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}


}
