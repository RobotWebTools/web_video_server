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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace web_video_server
{

/**
 * @brief A common interface for all streaming plugins.
 */
class StreamerInterface
{
public:
  virtual ~StreamerInterface() {}

  /**
   * @brief Starts the streaming process.
   */
  virtual void start() = 0;

  /**
   * @brief Returns true if the streamer is inactive and should be deleted.
   *
   * This could be because the connection was closed or snapshot was successfully sent (in case
   * of snapshot streamers).
   */
  virtual bool is_inactive() = 0;

  /**
   * @brief Restreams the last received image frame if older than max_age.
   */
  virtual void restream_frame(std::chrono::duration<double> max_age) = 0;

  /**
   * @brief Returns the topic being streamed.
   */
  virtual std::string get_topic() = 0;
};

/**
 * @brief A base class providing common functionality for streamers.
 */
class StreamerBase : public StreamerInterface
{
public:
  StreamerBase(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::WeakPtr node,
    std::string logger_name = "streamer");

  bool is_inactive() override
  {
    return inactive_;
  }

  std::string get_topic() override
  {
    return topic_;
  }

protected:
  rclcpp::Node::SharedPtr lock_node() const;

  async_web_server_cpp::HttpConnectionPtr connection_;
  async_web_server_cpp::HttpRequest request_;
  rclcpp::Node::WeakPtr node_;
  rclcpp::Logger logger_;
  bool inactive_;
  std::string topic_;
};

/**
 * @brief A factory interface for creating Streamer instances.
 */
class StreamerFactoryInterface
{
public:
  virtual ~StreamerFactoryInterface() = default;

  /**
   * @brief Returns the type of streamer created by this factory.
   *
   * This should match the "type" query parameter used to select the streamer.
   */
  virtual std::string get_type() = 0;

  /**
   * @brief Creates a new Streamer instance.
   * @param request The HTTP request that initiated the streamer.
   * @param connection The HTTP connection to use for streaming.
   * @param node The ROS2 node to use for subscribing to topics.
   * @return A shared pointer to the created Streamer instance.
   */
  virtual std::shared_ptr<StreamerInterface> create_streamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::WeakPtr node) = 0;

  /**
   * @brief Creates HTML code for embedding a viewer for this streamer.
   * @param request The HTTP request that initiated the viewer.
   */
  virtual std::string create_viewer(const async_web_server_cpp::HttpRequest & request);

  /**
   * @brief Returns a list of available topics that can be streamed by this streamer.
   * @param node The ROS2 node to use for discovering topics.
   * @return A vector of topic names.
   */
  virtual std::vector<std::string> get_available_topics(rclcpp::Node & node);
};

/**
 * @brief A factory interface for creating snapshot Streamer instances.
 */
class SnapshotStreamerFactoryInterface : public StreamerFactoryInterface {};

}  // namespace web_video_server
