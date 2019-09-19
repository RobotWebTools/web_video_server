#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>

namespace web_video_server
{

ImageStreamer::ImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
    request_(request), connection_(connection), nh_(nh), inactive_(false)
{
  topic_ = request.get_query_param_value_or_default("topic", "");
}

ImageStreamer::~ImageStreamer()
{
}

ImageTransportImageStreamer::ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
  ImageStreamer(request, connection, nh), it_(nh), initialized_(false)
{
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
  default_transport_ = request.get_query_param_value_or_default("default_transport", "raw");
}

ImageTransportImageStreamer::~ImageTransportImageStreamer()
{
}

void ImageTransportImageStreamer::start()
{
  image_transport::TransportHints hints(nh_.get(), default_transport_);
  auto tnat = nh_->get_topic_names_and_types();
  inactive_ = true;
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // explicitly avoid topics with more than one type
      break;
    }
    auto & topic_name = topic_and_types.first;
    if(topic_name == topic_ || (topic_name.find("/") == 0 && topic_name.substr(1) == topic_)){
      inactive_ = false;
      break;
    }
  }
  image_sub_ = it_.subscribe(topic_, 1, &ImageTransportImageStreamer::imageCallback, this, &hints);
}

void ImageTransportImageStreamer::initialize(const cv::Mat &)
{
}

void ImageTransportImageStreamer::restreamFrame(double max_age)
{
  if (inactive_ || !initialized_ )
    return;
  try {
    if ( last_frame + rclcpp::Duration(max_age) < nh_->now() ) {
      boost::mutex::scoped_lock lock(send_mutex_);
      sendImage(output_size_image, nh_->now() ); // don't update last_frame, it may remain an old value.
    }
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

void ImageTransportImageStreamer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  if (inactive_)
    return;

  cv::Mat img;
  try
  {
    if (msg->encoding.find("F") != std::string::npos)
    {
      // scale floating point images
      cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
      cv::Mat_<float> float_image = float_image_bridge;
      double max_val;
      cv::minMaxIdx(float_image, 0, &max_val);

      if (max_val > 0)
      {
        float_image *= (255 / max_val);
      }
      img = float_image;
    }
    else
    {
      // Convert to OpenCV native BGR color
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    int input_width = img.cols;
    int input_height = img.rows;

    if (output_width_ == -1)
      output_width_ = input_width;
    if (output_height_ == -1)
      output_height_ = input_height;

    if (invert_)
    {
      // Rotate 180 degrees
      cv::flip(img, img, false);
      cv::flip(img, img, true);
    }

    boost::mutex::scoped_lock lock(send_mutex_); // protects output_size_image
    if (output_width_ != input_width || output_height_ != input_height)
    {
      cv::Mat img_resized;
      cv::Size new_size(output_width_, output_height_);
      cv::resize(img, img_resized, new_size);
      output_size_image = img_resized;
    }
    else
    {
      output_size_image = img;
    }

    if (!initialized_)
    {
      initialize(output_size_image);
      initialized_ = true;
    }

    last_frame = nh_->now();
    sendImage(output_size_image, last_frame );

  }
  catch (cv_bridge::Exception &e)
  {
    // TODO THROTTLE with 30
    RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (cv::Exception &e)
  {
    // TODO THROTTLE with 30
    RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
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

}
