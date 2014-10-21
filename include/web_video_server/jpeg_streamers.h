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

#ifndef JPEG_STREAMERS_H_
#define JPEG_STREAMERS_H_

#include <image_transport/image_transport.h>
#include "web_video_server/image_streamer.h"
#include "web_video_server/http_server/http_request.hpp"
#include "web_video_server/http_server/http_connection.hpp"

namespace web_video_server
{

class MjpegStreamer : public ImageStreamer {
 public:
  MjpegStreamer(const http_server::HttpRequest& request,
		http_server::HttpConnectionPtr connection,
		image_transport::ImageTransport it);
 protected:
  virtual void sendImage(const cv::Mat&, const ros::Time& time);
 private:
  int quality_;
};


class JpegSnapshotStreamer : public ImageStreamer {
 public:
  JpegSnapshotStreamer(const http_server::HttpRequest& request,
		       http_server::HttpConnectionPtr connection,
		       image_transport::ImageTransport it);
 protected:
  virtual void sendImage(const cv::Mat&, const ros::Time& time);
 private:
  int quality_;
};

}


#endif

