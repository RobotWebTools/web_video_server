// Copyright (c) 2014, Worcester Polytechnic Institute
// Copyright (c) 2024-2025, The Robot Web Tools Contributors
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

#include "web_video_server/streamers/image_transport_streamer.hpp"

#include <chrono>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <boost/system/system_error.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include "cv_bridge/cv_bridge.h"
#else
#include "cv_bridge/cv_bridge.hpp"
#endif

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/transport_hints.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/logging.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/image.hpp"

#include "web_video_server/streamer.hpp"
#include "web_video_server/utils.hpp"

namespace web_video_server
{
namespace streamers
{

namespace
{

std::vector<std::string> get_image_topics(rclcpp::Node & node)
{
  std::vector<std::string> result;
  auto topic_names_and_types = node.get_topic_names_and_types();
  for (const auto & topic_and_types : topic_names_and_types) {
    for (const auto & type : topic_and_types.second) {
      if (type == "sensor_msgs/msg/Image") {
        result.push_back(topic_and_types.first);
        break;
      }
    }
  }
  return result;
}

}  // namespace

ImageTransportStreamerBase::ImageTransportStreamerBase(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node,
  std::string logger_name)
: StreamerBase(request, connection, node, logger_name), initialized_(false)
{
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
  default_transport_ = request.get_query_param_value_or_default("default_transport", "raw");
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
}

ImageTransportStreamerBase::~ImageTransportStreamerBase()
{
}

void ImageTransportStreamerBase::start()
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  image_transport::TransportHints hints(node.get(), default_transport_);
  auto tnat = node->get_topic_names_and_types();
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
    logger_, "Streaming topic %s with QoS profile %s", topic_.c_str(),
    qos_profile_name_.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name_);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      logger_,
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name_.c_str());
  }

  // Create subscriber
  image_sub_ = image_transport::create_subscription(
    node.get(), topic_,
    std::bind(&ImageTransportStreamerBase::image_callback, this, std::placeholders::_1),
    default_transport_, qos_profile.value());
}

void ImageTransportStreamerBase::initialize(const cv::Mat &)
{
}

void ImageTransportStreamerBase::restream_frame(std::chrono::duration<double>/* max_age */)
{
  if (inactive_ || !initialized_) {
    return;
  }

  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  try_send_image(output_size_image, last_frame_, *node);
}

void ImageTransportStreamerBase::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (inactive_) {
    return;
  }

  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  cv::Mat img;
  try {
    img = decode_image(msg);
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
  } catch (cv_bridge::Exception & e) {
    auto & clk = *node->get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (cv::Exception & e) {
    auto & clk = *node->get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "OpenCV exception: %s", e.what());
    inactive_ = true;
    return;
  }

  try_send_image(output_size_image, last_frame_, *node);
}

void ImageTransportStreamerBase::try_send_image(
  const cv::Mat & img,
  const std::chrono::steady_clock::time_point & /* time */,
  rclcpp::Node & node)
{
  try {
    std::scoped_lock lock(send_mutex_);
    send_image(img, std::chrono::steady_clock::now());
  } catch (boost::system::system_error & e) {
    // happens when client disconnects
    RCLCPP_DEBUG(logger_, "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (std::exception & e) {
    auto & clk = *node.get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (...) {
    auto & clk = *node.get_clock();
    RCLCPP_ERROR_THROTTLE(logger_, clk, 40, "exception");
    inactive_ = true;
    return;
  }
}

cv::Mat ImageTransportStreamerBase::decode_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->encoding.find("F") != std::string::npos) {
    // scale floating point images
    cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if (max_val > 0) {
      float_image *= (255 / max_val);
    }
    return float_image;
  } else {
    // Convert to OpenCV native BGR color
    return cv_bridge::toCvCopy(msg, "bgr8")->image;
  }
}

std::vector<std::string> ImageTransportStreamerFactoryBase::get_available_topics(
  rclcpp::Node & node)
{
  return get_image_topics(node);
}

std::vector<std::string> ImageTransportSnapshotStreamerFactoryBase::get_available_topics(
  rclcpp::Node & node)
{
  return get_image_topics(node);
}

}  // namespace streamers
}  // namespace web_video_server
