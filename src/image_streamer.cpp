// Copyright (c) 2014, Worcester Polytechnic Institute
// Copyright (c) 2024, The Robot Web Tools Contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "web_video_server/image_streamer.hpp"

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <iostream>

namespace web_video_server
{

ImageStreamer::ImageStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node)
: connection_(connection), request_(request), node_(node), inactive_(false)
{
  topic_ = request.get_query_param_value_or_default("topic", "");
}

ImageStreamer::~ImageStreamer()
{
}

ImageTransportImageStreamer::ImageTransportImageStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node)
: ImageStreamer(request, connection, node), it_(node), initialized_(false)
{
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
  default_transport_ = request.get_query_param_value_or_default("default_transport", "raw");
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

ImageTransportImageStreamer::~ImageTransportImageStreamer()
{
}

void ImageTransportImageStreamer::start()
{
  image_transport::TransportHints hints(node_.get(), default_transport_);
  auto tnat = node_->get_topic_names_and_types();
  inactive_ = true;
  for (auto topic_and_types : tnat) {
    if (topic_and_types.second.size() > 1) {
      // skip over topics with more than one type
      continue;
    }
    auto & topic_name = topic_and_types.first;
    if (topic_name == topic_ || (topic_name.find("/") == 0 && topic_name.substr(1) == topic_)) {
      inactive_ = false;
      break;
    }
  }

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    node_->get_logger(), "Streaming topic %s with QoS profile %s", topic_.c_str(),
    qos_profile_name_.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name_);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name_.c_str());
  }

  // Create subscriber
  image_sub_ = image_transport::create_subscription(
    node_.get(), topic_,
    std::bind(&ImageTransportImageStreamer::imageCallback, this, std::placeholders::_1),
    default_transport_, qos_profile.value());
}

void ImageTransportImageStreamer::initialize(const cv::Mat &)
{
}

void ImageTransportImageStreamer::restreamFrame(std::chrono::duration<double> max_age)
{
  if (inactive_ || !initialized_) {
    return;
  }
  try {
    if (last_frame_ + max_age < std::chrono::steady_clock::now()) {
      std::scoped_lock lock(send_mutex_);
      // don't update last_frame, it may remain an old value.
      sendImage(output_size_image, std::chrono::steady_clock::now());
    }
  } catch (boost::system::system_error & e) {
    // happens when client disconnects
    RCLCPP_DEBUG(node_->get_logger(), "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (std::exception & e) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (...) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception");
    inactive_ = true;
    return;
  }
}

cv::Mat ImageTransportImageStreamer::decodeImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->encoding.find("F") != std::string::npos) {
    // scale floating point images
    cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);
    return float_image;
  } else {
    // Convert to OpenCV native BGR color
    return cv_bridge::toCvCopy(msg, "bgr8")->image;
  }
}

void ImageTransportImageStreamer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (inactive_) {
    return;
  }

  cv::Mat img;
  try {
    img = decodeImage(msg);
    int input_width = img.cols;
    int input_height = img.rows;

    if (output_width_ == -1) {
      output_width_ = input_width;
    }
    if (output_height_ == -1) {
      output_height_ = input_height;
    }

    if (invert_) {
      // Rotate 180 degrees
      cv::flip(img, img, false);
      cv::flip(img, img, true);
    }

    std::scoped_lock lock(send_mutex_);  // protects output_size_image
    if (output_width_ != input_width || output_height_ != input_height) {
      cv::Mat img_resized;
      cv::Size new_size(output_width_, output_height_);
      cv::resize(img, img_resized, new_size);
      output_size_image = img_resized;
    } else {
      output_size_image = img;
    }

    if (!initialized_) {
      initialize(output_size_image);
      initialized_ = true;
    }

    last_frame_ = std::chrono::steady_clock::now();
    sendImage(output_size_image, last_frame_);
  } catch (cv_bridge::Exception & e) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (cv::Exception & e) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (boost::system::system_error & e) {
    // happens when client disconnects
    RCLCPP_DEBUG(node_->get_logger(), "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (std::exception & e) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (...) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception");
    inactive_ = true;
    return;
  }
}

}  // namespace web_video_server
