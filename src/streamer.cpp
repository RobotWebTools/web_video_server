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

#include "web_video_server/streamer.hpp"

#include <cerrno>
#include <vector>
#include <sstream>
#include <string>
#include <utility>

#include <sys/socket.h>
#include <netinet/tcp.h>

#include "rclcpp/node.hpp"
#include "rclcpp/logging.hpp"

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"

namespace web_video_server
{

StreamerBase::StreamerBase(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node,
  std::string logger_name)
: connection_(connection), request_(request), node_(std::move(node)),
  logger_(node_.lock()->get_logger().get_child(logger_name)), inactive_(false),
  topic_(request.get_query_param_value_or_default("topic", "")),
  client_id_(request.get_query_param_value_or_default("client_id", ""))
{
  // Enable TCP keepalive with aggressive probing so the kernel detects
  // dead connections even when the peer vanishes without sending FIN
  // (e.g. network drop, killed process).  After idle_s + cnt * intvl_s
  // seconds of silence the OS marks the socket dead, and our is_inactive()
  // recv(MSG_PEEK) check will see the error.
  try {
    int fd = connection_->socket().native_handle();
    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
    int idle_s = 5;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle_s, sizeof(idle_s));
    int intvl_s = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl_s, sizeof(intvl_s));
    int cnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
  } catch (...) {
    // Non-critical — fall through to the FIN-based detection.
  }
}

bool StreamerBase::is_inactive()
{
  if (inactive_) {
    return true;
  }
  // Detect closed TCP connections that would otherwise go unnoticed
  // when no messages arrive on the subscribed topic (nothing triggers
  // a write, so async_web_server_cpp never sees the broken pipe).
  if (connection_) {
    try {
      auto & socket = connection_->socket();
      if (!socket.is_open()) {
        inactive_ = true;
        return true;
      }
      char buf;
      ssize_t ret = ::recv(socket.native_handle(), &buf, 1, MSG_PEEK | MSG_DONTWAIT);
      if (ret == 0) {
        // Peer sent FIN — connection closed.
        inactive_ = true;
        return true;
      }
      if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        inactive_ = true;
        return true;
      }
    } catch (...) {
      inactive_ = true;
      return true;
    }
  }
  return false;
}

rclcpp::Node::SharedPtr StreamerBase::lock_node() const
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_WARN(logger_, "Unable to access node because the owning node has been destroyed");
  }
  return node;
}

std::string StreamerFactoryInterface::create_viewer(
  const async_web_server_cpp::HttpRequest & request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}

std::vector<std::string> StreamerFactoryInterface::get_available_topics(
  rclcpp::Node & /* node */)
{
  return {};
}

}  // namespace web_video_server
