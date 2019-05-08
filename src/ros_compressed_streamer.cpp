#include "web_video_server/ros_compressed_streamer.h"

namespace web_video_server
{

RosCompressedStreamer::RosCompressedStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageStreamer(request, connection, nh), stream_(connection)
{
  stream_.sendInitialHeader();
}

RosCompressedStreamer::~RosCompressedStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void RosCompressedStreamer::start() {
  std::string compressed_topic = topic_ + "/compressed";
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
    if(msg->format.find("jpeg") != std::string::npos) {
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
