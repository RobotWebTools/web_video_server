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

#pragma once

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavformat/avformat.h>
#include <libavutil/dict.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include <opencv2/core/mat.hpp>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "rclcpp/node.hpp"

#include "web_video_server/streamers/image_transport_streamer.hpp"

namespace web_video_server
{
namespace streamers
{

/**
 * @brief A common base class for all streaming plugins using image_transport to subscribe to image
 * topics and libav to encode and stream video.
 */
class LibavStreamerBase : public ImageTransportStreamerBase
{
public:
  LibavStreamerBase(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::WeakPtr node,
    std::string logger_name,
    const std::string & format_name,
    const std::string & codec_name,
    const std::string & content_type);

  ~LibavStreamerBase();

protected:
  virtual void initialize_encoder() = 0;
  virtual void send_image(const cv::Mat &, const std::chrono::steady_clock::time_point & time);
  virtual void initialize(const cv::Mat &);
  AVFormatContext * format_context_;
  const AVCodec * codec_;
  AVCodecContext * codec_context_;
  AVStream * video_stream_;

  AVDictionary * opt_;   // container format options

private:
  AVFrame * frame_;
  struct SwsContext * sws_context_;
  std::mutex encode_mutex_;
  bool first_image_received_;
  std::chrono::steady_clock::time_point first_image_time_;

  std::string format_name_;
  std::string codec_name_;
  std::string content_type_;
  int bitrate_;
  int qmin_;
  int qmax_;
  int gop_;

  uint8_t * io_buffer_;  // custom IO buffer
};

class LibavStreamerFactoryBase : public ImageTransportStreamerFactoryBase
{
public:
  virtual std::string create_viewer(const async_web_server_cpp::HttpRequest & request);
};

}  // namespace streamers
}  // namespace web_video_server
