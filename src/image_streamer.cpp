#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>

namespace web_video_server
{

ImageStreamer::ImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
    request_(request), connection_(connection), nh_(nh), inactive_(false)
{
  topic_ = request.get_query_param_value_or_default("topic", "");
}

ImageStreamer::~ImageStreamer()
{
}

ImageTransportImageStreamer::ImageTransportImageStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageStreamer(request, connection, nh), it_(nh), initialized_(false)
{
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
  default_transport_ = request.get_query_param_value_or_default("default_transport", "raw");

  min_v_ = request.get_query_param_value_or_default<float>("min", min_v_);
  max_v_ = request.get_query_param_value_or_default<float>("max", max_v_);
  colormap_ = request.get_query_param_value_or_default<int>("colormap", colormap_);

  if(std::isnan(min_v_) && !std::isnan(max_v_)) {
    min_v_ = 0;
  }
}

ImageTransportImageStreamer::~ImageTransportImageStreamer()
{
}

void ImageTransportImageStreamer::start()
{
  image_transport::TransportHints hints(default_transport_);
  ros::master::V_TopicInfo available_topics;
  ros::master::getTopics(available_topics);
  inactive_ = true;
  for (size_t it = 0; it<available_topics.size(); it++){
    std::string available_topic_name = available_topics[it].name;
    if(available_topic_name == topic_ || (available_topic_name.find("/") == 0 &&
                                          available_topic_name.substr(1) == topic_)) {
      inactive_ = false;
    }
  }
  image_sub_ = it_.subscribe(topic_, 1, &ImageTransportImageStreamer::imageCallback, this, hints);
}

void ImageTransportImageStreamer::initialize(const cv::Mat &)
{
}

void ImageTransportImageStreamer::restreamFrame(double max_age)
{
  if (inactive_ || !initialized_ )
    return;
  try {
    if ( last_frame + ros::Duration(max_age) < ros::Time::now() ) {
      boost::mutex::scoped_lock lock(send_mutex_);
      sendImage(output_size_image, ros::Time::now() ); // don't update last_frame, it may remain an old value.
    }
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

void ImageTransportImageStreamer::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (inactive_)
    return;

  cv::Mat img;
  try
  {
    if (msg->encoding.find("16") != std::string::npos || msg->encoding.find("F") != std::string::npos) {
      cv::Mat image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;

      double max_val, min_val;
      if(std::isnan(min_v_) || std::isnan(min_v_)) {
	cv::minMaxIdx(image_bridge, &min_val, &max_val);
      } else {
	min_val = min_v_;
	max_val = max_v_; 
      }

      float scale = 255. / (max_val - min_val);
      image_bridge.convertTo(img, CV_8U, scale, -min_val * scale);

      if(colormap_ >= 0) {
	cv::applyColorMap(img, img, colormap_);
      }
    }
    else
    {
      // Convert to OpenCV native BGR color
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    int input_width = img.cols;
    int input_height = img.rows;

    
    output_width_ = input_width;
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

    last_frame = ros::Time::now();
    sendImage(output_size_image, msg->header.stamp);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (cv::Exception &e)
  {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
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

}
