#ifndef WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_CONNECTION_HPP
#define WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_CONNECTION_HPP

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include "web_video_server/http_server/request_handler.hpp"
#include "web_video_server/http_server/http_request.hpp"
#include "web_video_server/http_server/http_request_parser.hpp"

namespace web_video_server
{
namespace http_server
{

// Represents a connection to a client
// To keep the connection alive keep a shared pointer to this object
class HttpConnection : public boost::enable_shared_from_this<HttpConnection>,
                       private boost::noncopyable
{
public:
  typedef boost::shared_ptr<void> ResourcePtr;

  explicit HttpConnection(boost::asio::io_service &io_service,
      HttpServerRequestHandler request_handler);

  boost::asio::ip::tcp::socket &socket();

  // Start async operation to read request
  void start();

  void write_and_clear(std::vector<unsigned char> &data);

  void write(const std::string &);

  void write(const boost::asio::const_buffer &buffer,
      ResourcePtr resource);

  void write(const std::vector<boost::asio::const_buffer> &buffer,
      ResourcePtr resource);

private:
  void handle_read(const boost::system::error_code &e,
      std::size_t bytes_transferred);

  // Must be called while holding write lock
  void write_pending();

  void handle_write(const boost::system::error_code &e,
      std::vector<boost::shared_ptr<void> > resources);

  boost::asio::io_service::strand strand_;
  boost::asio::ip::tcp::socket socket_;
  HttpServerRequestHandler request_handler_;
  boost::array<char, 8192> buffer_;
  HttpRequest request_;
  HttpRequestParser request_parser_;

  boost::mutex write_mutex_;
  bool write_in_progress_;
  std::vector<boost::asio::const_buffer> pending_write_buffers_;
  std::vector<ResourcePtr> pending_write_resources_;
  boost::system::error_code last_write_error_;
};

typedef boost::shared_ptr<HttpConnection> HttpConnectionPtr;

}
}

#endif
