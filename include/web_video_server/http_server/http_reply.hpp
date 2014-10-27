#ifndef WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_REPLY_HPP
#define WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_REPLY_HPP

#include <vector>
#include <string>
#include <boost/asio.hpp>
#include "web_video_server/http_server/http_header.hpp"
#include "web_video_server/http_server/http_connection.hpp"
#include "web_video_server/http_server/request_handler.hpp"

namespace web_video_server
{
namespace http_server
{

class ReplyBuilder;

// Utility methods for constructing replys
struct HttpReply
{
  enum status_type
  {
    ok = 200,
    created = 201,
    accepted = 202,
    no_content = 204,
    multiple_choices = 300,
    moved_permanently = 301,
    moved_temporarily = 302,
    not_modified = 304,
    bad_request = 400,
    unauthorized = 401,
    forbidden = 403,
    not_found = 404,
    internal_server_error = 500,
    not_implemented = 501,
    bad_gateway = 502,
    service_unavailable = 503
  } status;

  static std::vector<boost::asio::const_buffer> to_buffers(const std::vector<HttpHeader> &headers);

  static HttpServerRequestHandler stock_reply(status_type status);

  static ReplyBuilder builder(status_type status);
};


// Object to build and send a reply
class ReplyBuilder
{
public:
  ReplyBuilder(HttpReply::status_type status);

  ReplyBuilder &header(const std::string &name, const std::string &value);

  ReplyBuilder &header(const HttpHeader &header);

  ReplyBuilder &headers(const std::vector<HttpHeader> &headers);

  void write(HttpConnectionPtr connection);

private:
  HttpReply::status_type status_;
  boost::shared_ptr<std::vector<HttpHeader> > headers_;
};


// Request Handler that serves a a predefined response
class StaticHttpRequestHandler
{
public:
  StaticHttpRequestHandler(HttpReply::status_type status,
      const std::vector<HttpHeader> &headers,
      const std::string &content);

  void operator()(const HttpRequest &, boost::shared_ptr<HttpConnection>);

private:
  ReplyBuilder reply_builder_;
  const std::string content_string_;
};

}
}

#endif
