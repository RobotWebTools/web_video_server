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

#include "web_video_server/streamers/jpeg_streamers.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_reply.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "rclcpp/node.hpp"

#include "web_video_server/streamer.hpp"
#include "web_video_server/streamers/image_transport_streamer.hpp"

namespace web_video_server
{
namespace streamers
{

MjpegStreamer::MjpegStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::WeakPtr node)
: ImageTransportStreamerBase(request, connection, node, "mjpeg_streamer"),
  stream_(connection)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
  stream_.send_initial_header();
}

MjpegStreamer::~MjpegStreamer()
{
  this->inactive_ = true;
  std::scoped_lock lock(send_mutex_);  // protects send_image.
}

void MjpegStreamer::send_image(
  const cv::Mat & img,
  const std::chrono::steady_clock::time_point & time)
{
  std::vector<int> encode_params;
  encode_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uint8_t> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  stream_.send_part_and_clear(time, "image/jpeg", encoded_buffer);
}

std::shared_ptr<StreamerInterface> MjpegStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
{
  return std::make_shared<MjpegStreamer>(request, connection, node);
}

JpegSnapshotStreamer::JpegSnapshotStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
: ImageTransportStreamerBase(request, connection, node, "jpeg_snapshot_streamer")
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
}

JpegSnapshotStreamer::~JpegSnapshotStreamer()
{
  this->inactive_ = true;
  std::scoped_lock lock(send_mutex_);  // protects send_image.
}

void JpegSnapshotStreamer::send_image(
  const cv::Mat & img,
  const std::chrono::steady_clock::time_point & time)
{
  std::vector<int> encode_params;
  encode_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uint8_t> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  char stamp[20];
  snprintf(
    stamp, sizeof(stamp), "%.06lf",
    std::chrono::duration_cast<std::chrono::duration<double>>(time.time_since_epoch()).count());
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
  .header("Connection", "close")
  .header("Server", "web_video_server")
  .header(
    "Cache-Control",
    "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
  .header("X-Timestamp", stamp)
  .header("Pragma", "no-cache")
  .header("Content-type", "image/jpeg")
  .header("Access-Control-Allow-Origin", "*")
  .header("Content-Length", std::to_string(encoded_buffer.size()))
  .write(connection_);
  connection_->write_and_clear(encoded_buffer);
  inactive_ = true;
}

std::shared_ptr<StreamerInterface> JpegSnapshotStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
{
  return std::make_shared<JpegSnapshotStreamer>(request, connection, std::move(node));
}

}  // namespace streamers
}  // namespace web_video_server

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  web_video_server::streamers::MjpegStreamerFactory,
  web_video_server::StreamerFactoryInterface)
PLUGINLIB_EXPORT_CLASS(
  web_video_server::streamers::JpegSnapshotStreamerFactory,
  web_video_server::SnapshotStreamerFactoryInterface)
