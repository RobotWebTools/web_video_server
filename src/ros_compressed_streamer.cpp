#include "web_video_server/ros_compressed_streamer.h"

namespace web_video_server
{

RosCompressedStreamer::RosCompressedStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
  ImageStreamer(request, connection, nh), stream_(std::bind(&rclcpp::Node::now, nh), connection)
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
  image_sub_ = nh_->create_subscription<sensor_msgs::msg::CompressedImage>(
    compressed_topic, 1, std::bind(&RosCompressedStreamer::imageCallback, this, std::placeholders::_1));
}

void RosCompressedStreamer::restreamFrame(double max_age)
{
  if (inactive_ || (last_msg == 0))
    return;

  if ( last_frame + rclcpp::Duration(max_age) < nh_->now() ) {
    boost::mutex::scoped_lock lock(send_mutex_);
    sendImage(last_msg, nh_->now() ); // don't update last_frame, it may remain an old value.
  }
}

void RosCompressedStreamer::sendImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg,
                                      const rclcpp::Time &time) {
  try {
    std::string content_type;
    if(msg->format.find("jpeg") != std::string::npos) {
      content_type = "image/jpeg";
    }
    else if(msg->format.find("png") != std::string::npos) {
      content_type = "image/png";
    }
    else {
      RCLCPP_WARN(nh_->get_logger(), "Unknown ROS compressed image format: %s", msg->format.c_str());
      return;
    }

    stream_.sendPart(time, content_type, boost::asio::buffer(msg->data), msg);
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    RCLCPP_DEBUG(nh_->get_logger(), "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    // TODO THROTTLE with 30
    RCLCPP_ERROR(nh_->get_logger(), "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    // TODO THROTTLE with 30
    RCLCPP_ERROR(nh_->get_logger(), "exception");
    inactive_ = true;
    return;
  }
}


void RosCompressedStreamer::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
  boost::mutex::scoped_lock lock(send_mutex_); // protects last_msg and last_frame
  last_msg = msg;
  last_frame = rclcpp::Time(msg->header.stamp);
  sendImage(last_msg, last_frame);
}


boost::shared_ptr<ImageStreamer> RosCompressedStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
										 async_web_server_cpp::HttpConnectionPtr connection,
										 rclcpp::Node::SharedPtr nh)
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
