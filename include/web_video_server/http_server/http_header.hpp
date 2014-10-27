#ifndef WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_HEADER_HPP
#define WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_HEADER_HPP

#include <string>

namespace web_video_server
{
namespace http_server
{

struct HttpHeader
{
  HttpHeader()
  {
  }

  HttpHeader(std::string name, std::string value) : name(name), value(value)
  {
  }

  std::string name;
  std::string value;
};

}
}

#endif
