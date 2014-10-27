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

#ifndef WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_REQUEST_HPP
#define WEB_VIDEO_SERVER_HTTP_SERVER_HTTP_REQUEST_HPP

#include <string>
#include <vector>
#include <map>
#include <boost/lexical_cast.hpp>
#include "web_video_server/http_server/http_header.hpp"

namespace web_video_server {
namespace http_server {

struct HttpRequest {
  std::string method;
  std::string uri;
  int http_version_major;
  int http_version_minor;
  std::vector<HttpHeader> headers;

  std::string path;
  std::string query;
  std::map<std::string, std::string> query_params;
  bool has_query_param(const std::string& name) const;
  std::string get_query_param_value_or_default(const std::string& name,
					       const std::string& default_value) const;

  template <typename T>
  T get_query_param_value_or_default(const std::string& name,
					       const T& default_value) const {
    std::map<std::string, std::string>::const_iterator itr = query_params.find(name);
    if(itr != query_params.end()) {
      try {
	return boost::lexical_cast<T>(itr->second);
      }
      catch(const boost::bad_lexical_cast &) {
	return default_value;
      }
    }
    else{
      return default_value;
    }
}

  bool parse_uri();
};

}
}

#endif
