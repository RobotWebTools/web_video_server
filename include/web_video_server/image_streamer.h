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

#ifndef IMAGE_STREAMER_H_
#define IMAGE_STREAMER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "web_video_server/http_server/http_server.hpp"
#include "web_video_server/http_server/http_request.hpp"

namespace web_video_server
{

class ImageStreamer {
 public:
  ImageStreamer(const http_server::HttpRequest& request,
		http_server::HttpConnectionPtr connection,
		image_transport::ImageTransport it);
  void start();
  bool isInactive();
  std::string getTopic() { return topic_; };
 protected:
  virtual void sendImage(const cv::Mat&, const ros::Time& time) = 0;
  virtual void initialize(const cv::Mat&);
  http_server::HttpConnectionPtr connection_;
  http_server::HttpRequest request_;
  bool inactive_;
  image_transport::Subscriber image_sub_;
  std::string topic_;
  int output_width_;
  int output_height_;
  bool invert_;
 private:
  image_transport::ImageTransport it_;
  bool initialized_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

class ImageStreamerType {
 public:
  virtual boost::shared_ptr<ImageStreamer> create_streamer(const http_server::HttpRequest& request,
							   http_server::HttpConnectionPtr connection,
							   image_transport::ImageTransport it) = 0;
  virtual std::string create_viewer(const http_server::HttpRequest& request) = 0;
};

}


#endif
