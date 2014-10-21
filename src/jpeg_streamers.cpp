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

#include "web_video_server/jpeg_streamers.h"
#include "web_video_server/http_server/http_reply.hpp"

namespace web_video_server
{

MjpegStreamer::MjpegStreamer(const http_server::HttpRequest& request,
			       http_server::HttpConnectionPtr connection,
			       image_transport::ImageTransport it)
  : ImageStreamer(request, connection, it){
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);

  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("Pragma", "no-cache")
    .header("Content-type", "multipart/x-mixed-replace;boundary=--boundarydonotcross ")
    .header("Access-Control-Allow-Origin", "*")
    .write(connection);
  connection->write("--boundarydonotcross \r\n");
}
void MjpegStreamer::sendImage(const cv::Mat& img, const ros::Time& time) {
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  char stamp[20];
  sprintf(stamp, "%.06lf", time.toSec());
  boost::shared_ptr<std::vector<http_server::HttpHeader> > headers(new std::vector<http_server::HttpHeader>());
  headers->push_back(http_server::HttpHeader("Content-type", "image/jpeg"));
  headers->push_back(http_server::HttpHeader("X-Timestamp", stamp));
  headers->push_back(http_server::HttpHeader("Content-Length", boost::lexical_cast<std::string>(encoded_buffer.size())));
  connection_->write(http_server::HttpReply::to_buffers(*headers), headers);
  connection_->write_and_clear(encoded_buffer);
  connection_->write("\r\n--boundarydonotcross \r\n");
}

JpegSnapshotStreamer::JpegSnapshotStreamer(const http_server::HttpRequest& request,
			       http_server::HttpConnectionPtr connection,
			       image_transport::ImageTransport it)
  : ImageStreamer(request, connection, it){
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
}
void JpegSnapshotStreamer::sendImage(const cv::Mat& img, const ros::Time& time) {
  std::vector<int> encode_params;
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  char stamp[20];
  sprintf(stamp, "%.06lf", time.toSec());
  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("X-Timestamp", stamp)
    .header("Pragma", "no-cache")
    .header("Content-type", "image/jpeg")
    .header("Access-Control-Allow-Origin", "*")
    .header("Content-Length", boost::lexical_cast<std::string>(encoded_buffer.size()))
    .write(connection_);
  connection_->write_and_clear(encoded_buffer);
  inactive_ = true;
}



}
