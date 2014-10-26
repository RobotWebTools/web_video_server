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

#include "web_video_server/image_streamer.h"
#include <cv_bridge/cv_bridge.h>

namespace web_video_server
{

ImageStreamer::ImageStreamer(const http_server::HttpRequest& request,
			     http_server::HttpConnectionPtr connection,
			     image_transport::ImageTransport it)
  : request_(request), connection_(connection), it_(it),
    inactive_(false), initialized_(false) {
  topic_ = request.get_query_param_value_or_default("topic", "");
  output_width_ = request.get_query_param_value_or_default<int>("width", -1);
  output_height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
}
void ImageStreamer::start() {
  image_sub_ = it_.subscribe(topic_, 1, &ImageStreamer::imageCallback, this);
}
void ImageStreamer::initialize(const cv::Mat&) {}
void ImageStreamer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(inactive_)
    return;

  cv::Mat img;
  try {
    if(msg->encoding.find("F") != std::string::npos) {
      // scale floating point images
      cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
      cv::Mat_<float> float_image = float_image_bridge;
      double max_val;
      cv::minMaxIdx(float_image, 0, &max_val);

      if(max_val > 0) {
	float_image *= (255 / max_val);
      }
      img = float_image;
    }
    else {
      // Convert to OpenCV native BGR color
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    int input_width = img.cols;
    int input_height = img.rows;

    if(output_width_ == -1)
      output_width_ = input_width;
    if(output_height_ == -1)
      output_height_ = input_height;

    if(invert_) {
      // Rotate 180 degrees
      cv::flip(img, img, false);
      cv::flip(img, img, true);
    }

    cv::Mat output_size_image;
    if(output_width_ != input_width || output_height_ != input_height) {
      cv::Mat img_resized;
      cv::Size new_size(output_width_, output_height_);
      cv::resize(img, img_resized, new_size);
      output_size_image = img_resized;
    }
    else{
      output_size_image = img;
    }

    if(!initialized_){
      initialize(output_size_image);
      initialized_ = true;
    }
    sendImage(output_size_image, msg->header.stamp);

  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (cv::Exception& e) {
    ROS_ERROR_THROTTLE(30, "cv_bridge exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (boost::system::system_error& e) {
    // happens when client disconnects
    ROS_DEBUG("system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception& e) {
    ROS_ERROR_THROTTLE(30, "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...) {
    ROS_ERROR_THROTTLE(30, "exception");
    inactive_ = true;
    return;
  }
}
bool ImageStreamer::isInactive() {
  return inactive_;
}

}
