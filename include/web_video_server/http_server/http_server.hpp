#ifndef WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_SERVER_HPP
#define WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_SERVER_HPP

#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

#include "web_video_server/http_server/request_handler.hpp"
#include "web_video_server/http_server/http_connection.hpp"

namespace web_video_server
{
namespace http_server
{

class HttpServer : private boost::noncopyable
{
public:
  HttpServer(const std::string &address, const std::string &port,
      HttpServerRequestHandler request_handler, std::size_t thread_pool_size);

  void run();

  void stop();

private:
  void start_accept();

  void handle_accept(const boost::system::error_code &e);

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  std::size_t thread_pool_size_;
  std::vector<boost::shared_ptr<boost::thread> > threads_;
  boost::shared_ptr<HttpConnection> new_connection_;
  HttpServerRequestHandler request_handler_;
};

}
}

#endif
