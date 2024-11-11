#ifndef MULTIPART_STREAM_H_
#define MULTIPART_STREAM_H_

#include <ros/ros.h>
#include <async_web_server_cpp/http_connection.hpp>

#include <queue>

namespace web_video_server
{

struct PendingFooter {
  std::chrono::steady_clock::time_point timestamp;
  boost::weak_ptr<std::string> contents;
};

class MultipartStream {
public:
  MultipartStream(async_web_server_cpp::HttpConnectionPtr& connection,
                  const std::string& boundry="boundarydonotcross",
                  std::size_t max_queue_size=1);

  void sendInitialHeader();
  void sendPartHeader(const std::chrono::steady_clock::time_point &time, const std::string& type, size_t payload_size);
  void sendPartFooter(const std::chrono::steady_clock::time_point &time);
  void sendPartAndClear(const std::chrono::steady_clock::time_point &time, const std::string& type, std::vector<unsigned char> &data);
  void sendPart(const std::chrono::steady_clock::time_point &time, const std::string& type, const boost::asio::const_buffer &buffer,
		async_web_server_cpp::HttpConnection::ResourcePtr resource);

private:
  bool isBusy();

private:
  const std::size_t max_queue_size_;
  async_web_server_cpp::HttpConnectionPtr connection_;
  std::string boundry_;
  std::queue<PendingFooter> pending_footers_;
};

}

#endif
