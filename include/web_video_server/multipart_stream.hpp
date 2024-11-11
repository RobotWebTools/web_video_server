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

#include <queue>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "async_web_server_cpp/http_connection.hpp"

namespace web_video_server
{

struct PendingFooter
{
  std::chrono::steady_clock::time_point timestamp;
  std::weak_ptr<std::string> contents;
};

class MultipartStream
{
public:
  MultipartStream(
    async_web_server_cpp::HttpConnectionPtr & connection,
    const std::string & boundry = "boundarydonotcross",
    std::size_t max_queue_size = 1);

  void sendInitialHeader();
  void sendPartHeader(
    const std::chrono::steady_clock::time_point & time, const std::string & type,
    size_t payload_size);
  void sendPartFooter(const std::chrono::steady_clock::time_point & time);
  void sendPartAndClear(
    const std::chrono::steady_clock::time_point & time, const std::string & type,
    std::vector<unsigned char> & data);
  void sendPart(
    const std::chrono::steady_clock::time_point & time, const std::string & type,
    const boost::asio::const_buffer & buffer,
    async_web_server_cpp::HttpConnection::ResourcePtr resource);

private:
  bool isBusy();

private:
  const std::size_t max_queue_size_;
  async_web_server_cpp::HttpConnectionPtr connection_;
  std::string boundry_;
  std::queue<PendingFooter> pending_footers_;
};

}  // namespace web_video_server
