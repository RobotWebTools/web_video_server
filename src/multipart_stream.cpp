#include "web_video_server/multipart_stream.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

MultipartStream::MultipartStream(
    std::function<rclcpp::Time()> get_now,
    async_web_server_cpp::HttpConnectionPtr& connection,
    const std::string& boundry,
    std::size_t max_queue_size)
  : get_now_(get_now), connection_(connection), boundry_(boundry), max_queue_size_(max_queue_size)
{}

void MultipartStream::sendInitialHeader() {
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Content-type", "multipart/x-mixed-replace;boundary="+boundry_).header(
      "Access-Control-Allow-Origin", "*").write(connection_);
  connection_->write("--"+boundry_+"\r\n");
}

void MultipartStream::sendPartHeader(const rclcpp::Time &time, const std::string& type, size_t payload_size) {
  char stamp[20];
  sprintf(stamp, "%.06lf", time.seconds());
  std::shared_ptr<std::vector<async_web_server_cpp::HttpHeader> > headers(
      new std::vector<async_web_server_cpp::HttpHeader>());
  headers->push_back(async_web_server_cpp::HttpHeader("Content-type", type));
  headers->push_back(async_web_server_cpp::HttpHeader("X-Timestamp", stamp));
  headers->push_back(
      async_web_server_cpp::HttpHeader("Content-Length", boost::lexical_cast<std::string>(payload_size)));
  connection_->write(async_web_server_cpp::HttpReply::to_buffers(*headers), headers);
}

void MultipartStream::sendPartFooter(const rclcpp::Time &time) {
  std::shared_ptr<std::string> str(new std::string("\r\n--"+boundry_+"\r\n"));
  PendingFooter pf;
  pf.timestamp = time;
  pf.contents = str;
  connection_->write(boost::asio::buffer(*str), str);
  if (max_queue_size_ > 0) pending_footers_.push(pf);
}

void MultipartStream::sendPartAndClear(const rclcpp::Time &time, const std::string& type,
				       std::vector<unsigned char> &data) {
  if (!isBusy())
  {
    sendPartHeader(time, type, data.size());
    connection_->write_and_clear(data);
    sendPartFooter(time);
  }
}

void MultipartStream::sendPart(const rclcpp::Time &time, const std::string& type,
			       const boost::asio::const_buffer &buffer,
			       async_web_server_cpp::HttpConnection::ResourcePtr resource) {
  if (!isBusy())
  {
    sendPartHeader(time, type, boost::asio::buffer_size(buffer));
    connection_->write(buffer, resource);
    sendPartFooter(time);
  }
}

bool MultipartStream::isBusy() {
  rclcpp::Time currentTime = get_now_();
  while (!pending_footers_.empty())
  {
    if (pending_footers_.front().contents.expired()) {
      pending_footers_.pop();
    } else {
      rclcpp::Time footerTime = pending_footers_.front().timestamp;
      if ((currentTime - footerTime).seconds() > 0.5) {
        pending_footers_.pop();
      } else {
        break;
      }
    }
  }
  return !(max_queue_size_ == 0 || pending_footers_.size() < max_queue_size_);
}

}
