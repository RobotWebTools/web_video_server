/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Worcester Polytechnic Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Worcester Polytechnic Institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_REPLY_HPP
#define WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_REPLY_HPP

#include <vector>
#include <string>
#include <boost/asio.hpp>
#include "web_video_server/http_server/http_header.hpp"
#include "web_video_server/http_server/http_connection.hpp"
#include "web_video_server/http_server/request_handler.hpp"

namespace web_video_server {
namespace http_server {

class ReplyBuilder;

// Utility methods for constructing replys
struct HttpReply {
  enum status_type{
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

  static std::vector<boost::asio::const_buffer> to_buffers(const std::vector<HttpHeader>& headers);
  static HttpServerRequestHandler stock_reply(status_type status);
  static ReplyBuilder builder(status_type status);
};


// Request Handler that serves a a predefined response
class StaticHttpRequestHandler {
public:
  StaticHttpRequestHandler(HttpReply::status_type status,
			   const std::vector<HttpHeader>& headers,
			   const std::string& content);
  void operator()(const HttpRequest&, boost::shared_ptr<HttpConnection>);
private:
  const std::vector<HttpHeader> headers_;
  const std::string content_string_;
  std::vector<boost::asio::const_buffer> buffers_;
};

// Object to build and send a reply
class ReplyBuilder {
public:
  ReplyBuilder(HttpReply::status_type status);
  ReplyBuilder& header(const std::string& name, const std::string& value);
  ReplyBuilder& header(const HttpHeader& header);
  ReplyBuilder& headers(const std::vector<HttpHeader>& headers);
  void write(HttpConnectionPtr connection);
private:
  HttpReply::status_type status_;
  std::vector<HttpHeader> headers_;
};


}
}

#endif
