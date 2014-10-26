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

#ifndef LIBAV_STREAMERS_H_
#define LIBAV_STREAMERS_H_

#include <image_transport/image_transport.h>
#include "web_video_server/image_streamer.h"
#include "web_video_server/http_server/http_request.hpp"
#include "web_video_server/http_server/http_connection.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/intreadwrite.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>
}

namespace web_video_server
{

class LibavStreamer : public ImageStreamer {
 public:
  LibavStreamer(const http_server::HttpRequest& request,
		http_server::HttpConnectionPtr connection,
		image_transport::ImageTransport it,
		const std::string& format_name, const std::string& codec_name,
		const std::string& content_type);
  ~LibavStreamer();
 protected:
  virtual void sendImage(const cv::Mat&, const ros::Time& time);
  virtual void initialize(const cv::Mat&);
 private:
  AVOutputFormat* output_format_;
  AVFormatContext* format_context_;
  AVCodec* codec_;
  AVCodecContext* codec_context_;
  AVStream* video_stream_;
  AVFrame* frame_;
  AVPicture* picture_;
  AVPicture* tmp_picture_;
  struct SwsContext* sws_context_;
  ros::Time first_image_timestamp_;
  boost::mutex encode_mutex_;

  std::string format_name_;
  std::string codec_name_;
  std::string content_type_;
  int bitrate_;
  int qmin_;
  int qmax_;
  int gop_;
};

class LibavStreamerType : public ImageStreamerType{
 public:
  LibavStreamerType(const std::string& format_name, const std::string& codec_name,
		    const std::string& content_type);
  boost::shared_ptr<ImageStreamer> create_streamer(const http_server::HttpRequest& request,
							   http_server::HttpConnectionPtr connection,
							   image_transport::ImageTransport it);
  std::string create_viewer(const http_server::HttpRequest& request);
 private:
  const std::string format_name_;
  const std::string codec_name_;
  const std::string content_type_;
};


}


#endif

