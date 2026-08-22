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

#include "web_video_server/streamers/image_streamer.hpp"

#include <chrono>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <memory>

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
#include "rclcpp/node.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "web_video_server/streamer.hpp"
#include "web_video_server/subscriber.hpp"

namespace web_video_server
{
namespace streamers
{

ImageStreamerBase::ImageStreamerBase(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>> & subscriber_factories,
  rclcpp::Node::WeakPtr node,
  std::string logger_name)
: StreamerBase(request, connection, subscriber_factories, node, logger_name),
  initialized_(false)
{
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
}

ImageStreamerBase::~ImageStreamerBase()
{
}

void ImageStreamerBase::start()
{
  attach_subscriber(std::bind(&ImageStreamerBase::image_callback, this, std::placeholders::_1));
}

void ImageStreamerBase::initialize(const cv::Mat & /*img*/)
{
}

void ImageStreamerBase::restream_frame(std::chrono::duration<double>/* max_age */)
{
  if (inactive_ || !initialized_) {
    return;
  }

  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  try_send_image(output_size_image_, last_frame_, *node);
}

void ImageStreamerBase::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
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
    const int input_width = img.cols;
    const int input_height = img.rows;

    if (output_width_ == -1) {
      output_width_ = input_width;
    }
    if (output_height_ == -1) {
      output_height_ = input_height;
    }

    if (invert_) {
      // Rotate 180 degrees
      cv::flip(img, img, 0);
      cv::flip(img, img, 1);
    }

    const std::scoped_lock lock(send_mutex);  // protects output_size_image_
    if (output_width_ != input_width || output_height_ != input_height) {
      cv::Mat img_resized;
      const cv::Size new_size(output_width_, output_height_);
      cv::resize(img, img_resized, new_size);
      output_size_image_ = img_resized;
    } else {
      output_size_image_ = img;
    }

    if (!initialized_) {
      initialize(output_size_image_);
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

  try_send_image(output_size_image_, last_frame_, *node);
}

void ImageStreamerBase::try_send_image(
  const cv::Mat & img,
  const std::chrono::steady_clock::time_point & /* time */,
  rclcpp::Node & node)
{
  try {
    const std::scoped_lock lock(send_mutex);
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

cv::Mat ImageStreamerBase::decode_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->encoding.find("F") != std::string::npos) {
    // scale floating point images
    const cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if (max_val > 0) {
      float_image *= (255 / max_val);
    }
    return float_image;
  }
  // Convert to OpenCV native BGR color
  return cv_bridge::toCvCopy(msg, "bgr8")->image;
}

std::vector<std::string> ImageStreamerFactoryBase::get_available_topics(
  rclcpp::Node & node,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>> subscriber_factories)
{
  std::vector<std::string> results;

  for (auto subscriber: subscriber_factories) {
    std::vector<std::string> entries = subscriber.second->get_available_topics(node);
    results.insert(results.end(), entries.begin(), entries.end());
  }

  return results;
}

std::vector<std::string> ImageSnapshotStreamerFactoryBase::get_available_topics(
  rclcpp::Node & node,
  std::map<std::string, std::shared_ptr<SubscriberFactoryInterface>> subscriber_factories)
{
  std::vector<std::string> results;

  for (auto subscriber: subscriber_factories) {
    std::vector<std::string> entries = subscriber.second->get_available_topics(node);
    results.insert(results.end(), entries.begin(), entries.end());
  }

  return results;
}

}  // namespace streamers
}  // namespace web_video_server
