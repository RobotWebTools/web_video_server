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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_request_handler.hpp"
#include "async_web_server_cpp/http_server.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/timer.hpp"

#include "web_video_server/streamer.hpp"

namespace web_video_server
{

struct StreamerThreadWatch;

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

  bool handle_shutdown(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    const char * begin, const char * end);

  bool handle_list_streams(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    const char * begin, const char * end);

private:
  void restream_frames(std::chrono::duration<double> max_age);
  void activate_pending_streamers();
  void cleanup_inactive_streams();
  void resourse_management_timer_callback();
  void watchdog_loop();

  rclcpp::TimerBase::SharedPtr restream_timer_;
  rclcpp::TimerBase::SharedPtr resource_management_timer_;

  // Parameters
  double publish_rate_;
  int port_;
  std::string address_;
  bool verbose_;
  std::string default_stream_type_;
  std::string default_snapshot_type_;

  std::shared_ptr<async_web_server_cpp::HttpServer> server_;
  async_web_server_cpp::HttpRequestHandlerGroup handler_group_;

  std::vector<std::shared_ptr<StreamerInterface>> streamers_;
  std::vector<std::shared_ptr<StreamerInterface>> pending_streamers_;
  std::vector<std::thread> streamer_threads_;
  pluginlib::ClassLoader<StreamerFactoryInterface> streamer_factory_loader_;
  std::map<std::string, std::shared_ptr<StreamerFactoryInterface>> streamer_factories_;
  pluginlib::ClassLoader<SnapshotStreamerFactoryInterface> snapshot_streamer_factory_loader_;
  std::map<std::string, std::shared_ptr<StreamerFactoryInterface>> snapshot_streamer_factories_;
  std::mutex streamers_mutex_;
  std::mutex pending_mutex_;
  std::atomic<bool> creation_in_progress_{false};

  // Watchdog that exits the process when a streamer thread gets stuck
  // (FastDDS deadlock), so that it can be respawned.
  std::chrono::steady_clock::duration watchdog_timeout_;
  std::vector<std::weak_ptr<StreamerThreadWatch>> streamer_thread_watches_;
  std::mutex watchdog_mutex_;
  std::condition_variable watchdog_cv_;
  bool watchdog_stop_{false};
  std::thread watchdog_thread_;

  // How long a streamer may get no message from a matched publisher before
  // its subscription is recreated, zero to disable.
  std::chrono::steady_clock::duration resubscribe_timeout_;
};

}  // namespace web_video_server
