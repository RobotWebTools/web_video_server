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

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "web_video_server/streamer.hpp"

namespace web_video_server
{
namespace streamers
{

/**
 * @brief A common base class for all streaming plugins using image_transport to subscribe to image
 * topics.
 */
class ImageTransportStreamerBase : public StreamerBase
{
public:
  ImageTransportStreamerBase(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::WeakPtr node,
    std::string logger_name = "image_transport_streamer");
  virtual ~ImageTransportStreamerBase();

  virtual void start();
  virtual void restream_frame(std::chrono::duration<double> max_age);

protected:
  virtual cv::Mat decode_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  virtual void send_image(const cv::Mat &, const std::chrono::steady_clock::time_point & time) = 0;
  virtual void initialize(const cv::Mat &);

  image_transport::Subscriber image_sub_;
  int output_width_;
  int output_height_;
  bool invert_;
  std::string default_transport_;
  std::string qos_profile_name_;

  std::chrono::steady_clock::time_point last_frame_;
  cv::Mat output_size_image;
  std::mutex send_mutex_;

private:
  bool initialized_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void try_send_image(
    const cv::Mat & img, const std::chrono::steady_clock::time_point & time,
    rclcpp::Node & node);
};

class ImageTransportStreamerFactoryBase : public StreamerFactoryInterface
{
public:
  virtual std::vector<std::string> get_available_topics(rclcpp::Node & node);
};

class ImageTransportSnapshotStreamerFactoryBase : public SnapshotStreamerFactoryInterface
{
public:
  virtual std::vector<std::string> get_available_topics(rclcpp::Node & node);
};

}  // namespace streamers
}  // namespace web_video_server
