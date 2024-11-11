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

#include <map>
#include <memory>
#include <string>
#include <vector>

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include "cv_bridge/cv_bridge.h"
#else
#include "cv_bridge/cv_bridge.hpp"
#endif

#include "rclcpp/rclcpp.hpp"
#include "web_video_server/image_streamer.hpp"
#include "async_web_server_cpp/http_server.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"

namespace web_video_server
{

/**
 * @class WebVideoServer
 * @brief
 */
class WebVideoServer : public rclcpp::Node
{
public:
  /**
   * @brief  Constructor
   * @return
   */
  explicit WebVideoServer(const rclcpp::NodeOptions & options);

  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~WebVideoServer();

  bool handle_request(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    const char * begin, const char * end);

  bool handle_stream(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    const char * begin, const char * end);

  bool handle_stream_viewer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    const char * begin, const char * end);

  bool handle_snapshot(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    const char * begin, const char * end);

  bool handle_list_streams(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    const char * begin, const char * end);

private:
  void restreamFrames(std::chrono::duration<double> max_age);
  void cleanup_inactive_streams();

  rclcpp::TimerBase::SharedPtr cleanup_timer_;

  // Parameters
  int ros_threads_;
  double publish_rate_;
  int port_;
  std::string address_;
  bool verbose_;
  std::string default_stream_type_;

  std::shared_ptr<async_web_server_cpp::HttpServer> server_;
  async_web_server_cpp::HttpRequestHandlerGroup handler_group_;

  std::vector<std::shared_ptr<ImageStreamer>> image_subscribers_;
  std::map<std::string, std::shared_ptr<ImageStreamerType>> stream_types_;
  std::mutex subscriber_mutex_;
};

}  // namespace web_video_server
