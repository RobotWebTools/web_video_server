#include <boost/regex.hpp>
#include "web_video_server/http_server/request_handler.hpp"

namespace web_video_server
{
namespace http_server
{

RequestHandlerGroup::RequestHandlerGroup(HttpServerRequestHandler default_handler)
    : default_handler_(default_handler)
{
}

class PathMatcher
{
public:
  explicit PathMatcher(const std::string &path_regex_string)
      : path_regex_(boost::regex(path_regex_string))
  {
  }

  bool operator()(const HttpRequest &request)
  {
    return regex_match(request.path, path_regex_);
  }

private:
  const boost::regex path_regex_;
};

void RequestHandlerGroup::addHandlerForPath(const std::string &path_regex, HttpServerRequestHandler handler)
{
  addHandler(PathMatcher(path_regex), handler);
}

void RequestHandlerGroup::addHandler(HandlerPredicate predicate, HttpServerRequestHandler handler)
{
  handlers_.push_back(std::make_pair(predicate, handler));
}


void RequestHandlerGroup::operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection)
{
  for (int i = 0; i < handlers_.size(); ++i)
  {
    std::pair<HandlerPredicate, HttpServerRequestHandler> &handler = handlers_[i];
    if (handler.first(request))
    {
      handler.second(request, connection);
      return;
    }
  }
  default_handler_(request, connection);
}

}
}
