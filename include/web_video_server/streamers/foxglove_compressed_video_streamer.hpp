// Copyright (c) 2026, The Robot Web Tools Contributors
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

extern "C"
{
#include <libavcodec/avcodec.h>
}

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "foxglove_msgs/msg/compressed_video.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"

#include "web_video_server/streamer.hpp"

namespace web_video_server
{
namespace streamers
{

class FoxgloveCompressedVideoStreamer : public StreamerBase
{
public:
  FoxgloveCompressedVideoStreamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::WeakPtr node);
  ~FoxgloveCompressedVideoStreamer() override;

  void start() override;
  void restream_frame(std::chrono::duration<double> max_age) override;

private:
  using Message = foxglove_msgs::msg::CompressedVideo;
  void video_callback(Message::ConstSharedPtr msg);
  void reset_parser(const std::string & format);
  bool is_key_frame(const Message & msg);
  void send_http_header();
  void send_frame(Message::ConstSharedPtr msg, bool key_frame);

  rclcpp::Subscription<Message>::SharedPtr video_sub_;
  std::mutex send_mutex_;
  std::string qos_profile_name_;
  std::string parser_format_;
  const AVCodec * codec_{nullptr};
  AVCodecContext * codec_context_{nullptr};
  AVCodecParserContext * parser_{nullptr};
};

class FoxgloveCompressedVideoStreamerFactory : public StreamerFactoryInterface
{
public:
  std::string get_type() override {return "foxglove_compressed_video";}
  std::shared_ptr<StreamerInterface> create_streamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::WeakPtr node) override;
  std::string create_viewer(const async_web_server_cpp::HttpRequest & request) override;
  std::vector<std::string> get_available_topics(rclcpp::Node & node) override;
};

}  // namespace streamers
}  // namespace web_video_server
