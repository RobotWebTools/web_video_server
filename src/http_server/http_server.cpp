#include "web_video_server/http_server/http_server.hpp"
#include "web_video_server/http_server/http_reply.hpp"

namespace web_video_server
{
namespace http_server
{

HttpServer::HttpServer(const std::string &address, const std::string &port,
    HttpServerRequestHandler request_handler, std::size_t thread_pool_size)
    : acceptor_(io_service_), thread_pool_size_(thread_pool_size), request_handler_(request_handler)
{

  boost::asio::ip::tcp::resolver resolver(io_service_);
  boost::asio::ip::tcp::resolver::query query(address, port);
  boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);
  acceptor_.open(endpoint.protocol());
  acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
  acceptor_.bind(endpoint);
  acceptor_.listen();
}


void HttpServer::run()
{
  start_accept();
  for (std::size_t i = 0; i < thread_pool_size_; ++i)
  {
    boost::shared_ptr<boost::thread> thread(new boost::thread(
        boost::bind(&boost::asio::io_service::run, &io_service_)));
    threads_.push_back(thread);
  }
}

void HttpServer::start_accept()
{
  new_connection_.reset(new HttpConnection(io_service_, request_handler_));
  acceptor_.async_accept(new_connection_->socket(),
      boost::bind(&HttpServer::handle_accept, this,
          boost::asio::placeholders::error));
}

void HttpServer::handle_accept(const boost::system::error_code &e)
{
  if (!e)
  {
    new_connection_->start();
  }

  start_accept();
}

void HttpServer::stop()
{
  io_service_.stop();
  // Wait for all threads in the pool to exit.
  for (std::size_t i = 0; i < threads_.size(); ++i)
    threads_[i]->join();
}

}
}
