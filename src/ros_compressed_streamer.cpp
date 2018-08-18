#include "web_video_server/ros_compressed_streamer.h"

namespace web_video_server
{

RosCompressedStreamer::RosCompressedStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
  ImageStreamer(request, connection, nh), stream_(connection)
{
  stream_.sendInitialHeader();
}

void RosCompressedStreamer::start() {
  std::string compressed_topic = topic_ + "/compressed";
  image_sub_ = nh_->create_subscription<sensor_msgs::msg::CompressedImage>(
    compressed_topic, std::bind(&RosCompressedStreamer::imageCallback, this, std::placeholders::_1), 1);
}

void RosCompressedStreamer::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
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

    stream_.sendPart(rclcpp::Time(msg->header.stamp), content_type, boost::asio::buffer(msg->data), msg);
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
