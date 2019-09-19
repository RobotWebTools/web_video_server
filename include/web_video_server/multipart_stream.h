#ifndef MULTIPART_STREAM_H_
#define MULTIPART_STREAM_H_

#include <rclcpp/rclcpp.hpp>
#include <async_web_server_cpp/http_connection.hpp>

#include <queue>

namespace web_video_server
{

struct PendingFooter {
  rclcpp::Time timestamp;
  std::weak_ptr<std::string> contents;
};

class MultipartStream {
public:
  MultipartStream(std::function<rclcpp::Time()> get_now,
                  async_web_server_cpp::HttpConnectionPtr& connection,
                  const std::string& boundry="boundarydonotcross",
                  std::size_t max_queue_size=1);

  void sendInitialHeader();
  void sendPartHeader(const rclcpp::Time &time, const std::string& type, size_t payload_size);
  void sendPartFooter(const rclcpp::Time &time);
  void sendPartAndClear(const rclcpp::Time &time, const std::string& type, std::vector<unsigned char> &data);
  void sendPart(const rclcpp::Time &time, const std::string& type, const boost::asio::const_buffer &buffer,
		async_web_server_cpp::HttpConnection::ResourcePtr resource);

private:
  bool isBusy();

private:
  std::function<rclcpp::Time()> get_now_;
  const std::size_t max_queue_size_;
  async_web_server_cpp::HttpConnectionPtr connection_;
  std::string boundry_;
  std::queue<PendingFooter> pending_footers_;
};

}

#endif
