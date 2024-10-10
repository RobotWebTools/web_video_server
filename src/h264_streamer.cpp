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

#include "web_video_server/h264_streamer.hpp"

namespace web_video_server
{

H264Streamer::H264Streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node)
: LibavStreamer(request, connection, node, "mp4", "libx264", "video/mp4")
{
  /* possible quality presets:
   * ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow, placebo
   * no latency improvements observed with ultrafast instead of medium
   */
  preset_ = request.get_query_param_value_or_default("preset", "ultrafast");
}

H264Streamer::~H264Streamer()
{
}

void H264Streamer::initializeEncoder()
{
  av_opt_set(codec_context_->priv_data, "preset", preset_.c_str(), 0);
  av_opt_set(codec_context_->priv_data, "tune", "zerolatency", 0);
  av_opt_set_int(codec_context_->priv_data, "crf", 20, 0);
  av_opt_set_int(codec_context_->priv_data, "bufsize", 100, 0);
  av_opt_set_int(codec_context_->priv_data, "keyint", 30, 0);
  av_opt_set_int(codec_context_->priv_data, "g", 1, 0);

  // container format options
  if (!strcmp(format_context_->oformat->name, "mp4")) {
    // set up mp4 for streaming (instead of seekable file output)
    av_dict_set(&opt_, "movflags", "+frag_keyframe+empty_moov+faststart", 0);
  }
}

H264StreamerType::H264StreamerType()
: LibavStreamerType("mp4", "libx264", "video/mp4")
{
}

std::shared_ptr<ImageStreamer> H264StreamerType::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::SharedPtr node)
{
  return std::make_shared<H264Streamer>(request, connection, node);
}

}  // namespace web_video_server
