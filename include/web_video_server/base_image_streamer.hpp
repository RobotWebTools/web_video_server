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

#include <chrono>
#include <memory>
#include <string>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "image_transport/subscriber.hpp"
#include "rclcpp/node.hpp"

namespace web_video_server
{

class BaseImageStreamer
{
public:
  BaseImageStreamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node);

  virtual void start() = 0;
  virtual ~BaseImageStreamer();

  bool isInactive()
  {
    return inactive_;
  }

  /**
   * Restreams the last received image frame if older than max_age.
   */
  virtual void restreamFrame(std::chrono::duration<double> max_age) = 0;

  std::string getTopic()
  {
    return topic_;
  }

protected:
  async_web_server_cpp::HttpConnectionPtr connection_;
  async_web_server_cpp::HttpRequest request_;
  rclcpp::Node::SharedPtr node_;
  bool inactive_;
  std::string topic_;
};

class BaseImageStreamerFactory
{
public:
  virtual std::string get_type() = 0;

  virtual std::shared_ptr<BaseImageStreamer> create_streamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node) = 0;

  virtual std::string create_viewer(const async_web_server_cpp::HttpRequest & request);
};

class BaseSnapshotStreamerFactory : public BaseImageStreamerFactory {};

}  // namespace web_video_server
