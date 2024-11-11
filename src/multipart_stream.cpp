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

#include "web_video_server/multipart_stream.hpp"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

MultipartStream::MultipartStream(
  async_web_server_cpp::HttpConnectionPtr & connection,
  const std::string & boundry,
  std::size_t max_queue_size)
: connection_(connection), boundry_(boundry), max_queue_size_(max_queue_size)
{}

void MultipartStream::sendInitialHeader()
{
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
  .header("Connection", "close")
  .header("Server", "web_video_server")
  .header(
    "Cache-Control",
    "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
  .header("Pragma", "no-cache")
  .header("Content-type", "multipart/x-mixed-replace;boundary=" + boundry_)
  .header("Access-Control-Allow-Origin", "*")
  .write(connection_);
  connection_->write("--" + boundry_ + "\r\n");
}

void MultipartStream::sendPartHeader(
  const std::chrono::steady_clock::time_point & time, const std::string & type,
  size_t payload_size)
{
  char stamp[20];
  snprintf(
    stamp, sizeof(stamp), "%.06lf",
    std::chrono::duration_cast<std::chrono::duration<double>>(time.time_since_epoch()).count());
  std::shared_ptr<std::vector<async_web_server_cpp::HttpHeader>> headers(
    new std::vector<async_web_server_cpp::HttpHeader>());
  headers->push_back(async_web_server_cpp::HttpHeader("Content-type", type));
  headers->push_back(async_web_server_cpp::HttpHeader("X-Timestamp", stamp));
  headers->push_back(
    async_web_server_cpp::HttpHeader(
      "Content-Length", std::to_string(payload_size)));
  connection_->write(async_web_server_cpp::HttpReply::to_buffers(*headers), headers);
}

void MultipartStream::sendPartFooter(const std::chrono::steady_clock::time_point & time)
{
  std::shared_ptr<std::string> str(new std::string("\r\n--" + boundry_ + "\r\n"));
  PendingFooter pf;
  pf.timestamp = time;
  pf.contents = str;
  connection_->write(boost::asio::buffer(*str), str);
  if (max_queue_size_ > 0) {pending_footers_.push(pf);}
}

void MultipartStream::sendPartAndClear(
  const std::chrono::steady_clock::time_point & time, const std::string & type,
  std::vector<unsigned char> & data)
{
  if (!isBusy()) {
    sendPartHeader(time, type, data.size());
    connection_->write_and_clear(data);
    sendPartFooter(time);
  }
}

void MultipartStream::sendPart(
  const std::chrono::steady_clock::time_point & time, const std::string & type,
  const boost::asio::const_buffer & buffer,
  async_web_server_cpp::HttpConnection::ResourcePtr resource)
{
  if (!isBusy()) {
    sendPartHeader(time, type, boost::asio::buffer_size(buffer));
    connection_->write(buffer, resource);
    sendPartFooter(time);
  }
}

bool MultipartStream::isBusy()
{
  auto current_time = std::chrono::steady_clock::now();
  while (!pending_footers_.empty()) {
    if (pending_footers_.front().contents.expired()) {
      pending_footers_.pop();
    } else {
      auto footer_time = pending_footers_.front().timestamp;
      if (std::chrono::duration_cast<std::chrono::duration<double>>(
          (current_time - footer_time)).count() > 0.5)
      {
        pending_footers_.pop();
      } else {
        break;
      }
    }
  }
  return !(max_queue_size_ == 0 || pending_footers_.size() < max_queue_size_);
}

}  // namespace web_video_server
