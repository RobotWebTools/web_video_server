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

#pragma once

#include <memory>
#include <string>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "web_video_server/image_streamer.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.hpp"

namespace web_video_server
{

class RosCompressedStreamer : public ImageStreamer
{
public:
  RosCompressedStreamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node);
  ~RosCompressedStreamer();
  virtual void start();
  virtual void restreamFrame(std::chrono::duration<double> max_age);

protected:
  virtual void sendImage(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg,
    const std::chrono::steady_clock::time_point & time);

private:
  void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  MultipartStream stream_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  std::chrono::steady_clock::time_point last_frame_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr last_msg;
  std::mutex send_mutex_;
  std::string qos_profile_name_;
};

class RosCompressedStreamerType : public ImageStreamerType
{
public:
  std::shared_ptr<ImageStreamer> create_streamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node);
  std::string create_viewer(const async_web_server_cpp::HttpRequest & request);
};

}  // namespace web_video_server
