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

#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include "web_video_server/http_server/http_request.hpp"

namespace web_video_server {
namespace http_server {

static boost::regex uri_regex("(.*?)(?:\\?(.*?))?");

bool HttpRequest::parse_uri(){
  boost::smatch match;
  if(regex_match(uri, match, uri_regex)){
    path.assign(match[1].first, match[1].second);
    if(match[2].matched){
      query.assign(match[2].first, match[2].second);

      std::vector<std::string> pair_strings;
      boost::split(pair_strings, query, boost::is_any_of("&"));
      BOOST_FOREACH(const std::string& pair_string, pair_strings) {
	std::vector<std::string> pair_data;
	const int eq_index = pair_string.find_first_of('=');
	if(eq_index == std::string::npos) {
	  if(pair_string.size() > 0) {
	    query_params[pair_string] = "";
	  }
	}
	else {
	  query_params[pair_string.substr(0, eq_index)] = pair_string.substr(eq_index + 1);
	}
      }
    }
    return true;
  }
  else{
    return false;
  }

}
bool HttpRequest::has_query_param(const std::string& name) const {
  std::map<std::string, std::string>::const_iterator itr = query_params.find(name);
  return itr != query_params.end();
}

std::string HttpRequest::get_query_param_value_or_default(const std::string& name,
							  const std::string& default_value) const {
  std::map<std::string, std::string>::const_iterator itr = query_params.find(name);
  if(itr != query_params.end()) {
    return itr->second;
  }
  else{
    return default_value;
  }
}



}
}
