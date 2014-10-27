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


#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include "web_video_server/http_server/http_reply.hpp"
#include "web_video_server/http_server/http_connection.hpp"

namespace web_video_server {
namespace http_server {


HttpConnection::HttpConnection(boost::asio::io_service& io_service,
			       HttpServerRequestHandler handler)
  : strand_(io_service),
    socket_(io_service),
    request_handler_(handler),
    write_in_progress_(false){}

boost::asio::ip::tcp::socket& HttpConnection::socket() {
  return socket_;
}

void HttpConnection::start() {
  socket_.async_read_some(boost::asio::buffer(buffer_),
			  strand_.wrap(
				       boost::bind(&HttpConnection::handle_read, shared_from_this(),
						   boost::asio::placeholders::error,
						   boost::asio::placeholders::bytes_transferred)));
}

void HttpConnection::handle_read(const boost::system::error_code& e,
				 std::size_t bytes_transferred) {
  if (!e) {
    boost::tribool result;
    boost::tie(result, boost::tuples::ignore) = request_parser_.parse(
								      request_, buffer_.data(), buffer_.data() + bytes_transferred);

    if (result) {
      request_.parse_uri();
      try {
	request_handler_(request_, shared_from_this());
      } catch(...){
	// error constructing request
	// just kill the connection as the handler may have already started writing stuff out
      }
   }
    else if (!result) {
      HttpReply::stock_reply(HttpReply::bad_request)(request_, shared_from_this());
    }
    else {
      socket_.async_read_some(boost::asio::buffer(buffer_),
			      strand_.wrap(
					   boost::bind(&HttpConnection::handle_read, shared_from_this(),
						       boost::asio::placeholders::error,
						       boost::asio::placeholders::bytes_transferred)));
    }
  }
}



void HttpConnection::write_and_clear(std::vector<unsigned char>& data) {
  boost::shared_ptr<std::vector<unsigned char> > buffer(new std::vector<unsigned char>());
  buffer->swap(data);
  write(boost::asio::buffer(*buffer), buffer);
}
void HttpConnection::write(const std::string& content) {
  boost::shared_ptr<std::string> str(new std::string(content));
  write(boost::asio::buffer(*str), str);
}

void HttpConnection::write(const boost::asio::const_buffer& buffer,
			   ResourcePtr resource){
  boost::mutex::scoped_lock lock(write_mutex_);
  pending_write_buffers_.push_back(buffer);
  if(resource)
    pending_write_resources_.push_back(resource);
  if(!write_in_progress_)
    write_pending();
}
void HttpConnection::write(const std::vector<boost::asio::const_buffer>& buffers,
			   ResourcePtr resource){
  boost::mutex::scoped_lock lock(write_mutex_);
  pending_write_buffers_.insert(pending_write_buffers_.end(), buffers.begin(), buffers.end());
  if(resource)
    pending_write_resources_.push_back(resource);
  if(!write_in_progress_)
    write_pending();
}


// Must be called while holding write lock
void HttpConnection::write_pending() {
  if(last_write_error_){
    boost::throw_exception(boost::system::system_error(last_write_error_));
  }
  write_in_progress_ = true;
  boost::asio::async_write(socket_, pending_write_buffers_,
			   boost::bind(&HttpConnection::handle_write, shared_from_this(),
				       boost::asio::placeholders::error,
				       pending_write_resources_));
  pending_write_buffers_.clear();
  pending_write_resources_.clear();
}
void HttpConnection::handle_write(const boost::system::error_code& e,
				  std::vector<ResourcePtr> resources) {
  boost::mutex::scoped_lock lock(write_mutex_);
  write_in_progress_ = false;
  if(!e) {
    if(!pending_write_buffers_.empty()){
      write_pending();
    }
  }
  else {
    last_write_error_ = e;
  }
}



}
}
