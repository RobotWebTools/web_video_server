#ifndef WEB_VIDEO_SERVER_HTTP_SERVER_REQUEST_HANDLER_HPP
#define WEB_VIDEO_SERVER_HTTP_SERVER_REQUEST_HANDLER_HPP

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "web_video_server/http_server/http_request.hpp"

namespace web_video_server
{
namespace http_server
{

class HttpConnection;

typedef boost::function<void(const HttpRequest &, boost::shared_ptr<HttpConnection>)> HttpServerRequestHandler;

class RequestHandlerGroup
{
public:
  typedef boost::function<bool(const HttpRequest &)> HandlerPredicate;

  RequestHandlerGroup(HttpServerRequestHandler default_handler);

  void addHandlerForPath(const std::string &path_regex, HttpServerRequestHandler handler);

  void addHandler(HandlerPredicate predicate, HttpServerRequestHandler handler);

  void operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection);

private:
  HttpServerRequestHandler default_handler_;
  std::vector<std::pair<HandlerPredicate, HttpServerRequestHandler> > handlers_;
};

}
}

#endif
