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

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include "web_video_server/web_video_server.h"
#include "web_video_server/http_server/http_reply.hpp"


namespace web_video_server
{

ImageStreamer::ImageStreamer(const http_server::HttpRequest& request,
			     http_server::HttpConnectionPtr connection,
			     image_transport::ImageTransport it)
    : connection_(connection), inactive_(false) {
  topic_ = request.get_query_param_value_or_default("topic", "");
  width_ = request.get_query_param_value_or_default<int>("width", -1);
  height_ = request.get_query_param_value_or_default<int>("height", -1);
  invert_ = request.has_query_param("invert");
  image_sub_ = it.subscribe(topic_, 1, &ImageStreamer::imageCallback, this);
}
void ImageStreamer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if(inactive_)
    return;

  cv::Mat img_orig;
  try {
    if(msg->encoding.find("F") != std::string::npos) {
      // scale floating point images
      cv::Mat float_image_bridge = cv_bridge::toCvShare(msg, msg->encoding)->image;
      cv::Mat_<float> float_image = float_image_bridge;
      double max_val;
      cv::minMaxIdx(float_image, 0, &max_val);

      if(max_val > 0) {
	float_image *= (255 / max_val);
      }
      img_orig = float_image;
    }
    else {
      // Convert to OpenCV native BGR color
      img_orig = cv_bridge::toCvShare(msg, "bgr8")->image;
    }

    cv::Mat img = img_orig.clone();

    if(invert_) {
      // Rotate 180 degrees
      cv::flip(img, img, false);
      cv::flip(img, img, true);
    }

    if(width_ > 0 && height_ > 0) {
      cv::Mat img_resized;
      cv::Size new_size(width_, height_);
      cv::resize(img, img_resized, new_size);
      sendImage(img_resized, msg->header.stamp);
    }
    else{
      sendImage(img, msg->header.stamp);
    }
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

static void ros_connection_logger(http_server::HttpServerRequestHandler forward,
				  const http_server::HttpRequest& request,
				  http_server::HttpConnectionPtr connection) {
  ROS_INFO_STREAM("Handling Request: " << request.uri);
  try {
    forward(request, connection);
  } catch(std::exception& e) {
    ROS_WARN_STREAM("Error Handling Request: " << e.what());
  }
}


WebVideoServer::WebVideoServer(ros::NodeHandle& nh, ros::NodeHandle& private_nh) :
  nh_(nh), image_transport_(nh),
  handler_group_(http_server::HttpReply::stock_reply(http_server::HttpReply::not_found)) {
  cleanup_timer_ = nh.createTimer(ros::Duration(0.5), boost::bind(&WebVideoServer::cleanup_inactive_streams, this));

  int port;
  private_nh.param("port", port, 8080);

  int server_threads;
  private_nh.param("server_threads", server_threads, 1);

  private_nh.param("ros_threads", ros_threads_, 2);

  handler_group_.addHandlerForPath("/", boost::bind(&WebVideoServer::handle_list_streams, this, _1, _2));
  handler_group_.addHandlerForPath("/stream", boost::bind(&WebVideoServer::handle_stream, this, _1, _2));
  handler_group_.addHandlerForPath("/stream_viewer", boost::bind(&WebVideoServer::handle_stream_viewer, this, _1, _2));
  handler_group_.addHandlerForPath("/snapshot", boost::bind(&WebVideoServer::handle_snapshot, this, _1, _2));

  server_.reset(new http_server::HttpServer("0.0.0.0", boost::lexical_cast<std::string>(port),
					    boost::bind(ros_connection_logger, handler_group_, _1, _2), server_threads));
}

WebVideoServer::~WebVideoServer() {
}

void WebVideoServer::spin() {
  server_->run();
  ROS_INFO("Waiting For connections");
  ros::MultiThreadedSpinner spinner(ros_threads_);
  spinner.spin();
  server_->stop();
}

void WebVideoServer::cleanup_inactive_streams(){
  boost::mutex::scoped_lock lock(subscriber_mutex_, boost::try_to_lock);
  if(lock) {
    typedef std::vector<boost::shared_ptr<ImageStreamer> >::iterator itr_type;
    itr_type new_end
      = std::remove_if(image_subscribers_.begin(), image_subscribers_.end(), boost::bind(&ImageStreamer::isInactive, _1));
    for(itr_type itr = new_end; itr < image_subscribers_.end(); ++ itr){
      ROS_INFO_STREAM("Removed Stream: " << (*itr)->getTopic());
    }
    image_subscribers_.erase(new_end,
			     image_subscribers_.end());
  }
}


void WebVideoServer::handle_stream(const http_server::HttpRequest& request,
				http_server::HttpConnectionPtr connection) {
  boost::mutex::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(boost::shared_ptr<ImageStreamer>(new MjpegStreamer(request, connection, image_transport_)));
}

void WebVideoServer::handle_snapshot(const http_server::HttpRequest& request,
				  http_server::HttpConnectionPtr connection) {
  boost::mutex::scoped_lock lock(subscriber_mutex_);
  image_subscribers_.push_back(boost::shared_ptr<ImageStreamer>(new JpegSnapshotStreamer(request, connection, image_transport_)));
}

void WebVideoServer::handle_stream_viewer(const http_server::HttpRequest& request,
				      http_server::HttpConnectionPtr connection) {
  std::string topic = request.get_query_param_value_or_default("topic", "");
  std::string width = request.get_query_param_value_or_default("width", "640");
  std::string height = request.get_query_param_value_or_default("height", "480");

  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Content-type", "text/html;")
    .write(connection);

  std::stringstream ss;
  ss << "<html><head><title>" << topic << "</title></head><body>";
  ss << "<h1>" << topic << "</h1>";
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\" width=\"" << width << "\" height=\"" << height << "\"></img>";
  ss << "</body></html>";
  connection->write(ss.str());
}

void WebVideoServer::handle_list_streams(const http_server::HttpRequest& request,
				      http_server::HttpConnectionPtr connection) {
  std::string image_message_type = ros::message_traits::datatype<sensor_msgs::Image>();
  std::string camera_info_message_type = ros::message_traits::datatype<sensor_msgs::CameraInfo>();

  ros::master::V_TopicInfo topics;
  ros::master::getTopics( topics );
  ros::master::V_TopicInfo::iterator it;
  std::vector<std::string> image_topics;
  std::vector<std::string> camera_info_topics;
  for( it = topics.begin(); it != topics.end(); ++it ) {
    const ros::master::TopicInfo& topic = *it;
    if (topic.datatype == image_message_type) {
	image_topics.push_back(topic.name);
    }
    else if (topic.datatype == camera_info_message_type) {
	camera_info_topics.push_back(topic.name);
    }
  }


  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("Pragma", "no-cache")
    .header("Content-type", "text/html;")
    .write(connection);

  connection->write("<html>"
		    "<head><title>ROS Image Topic List</title></head>"
		    "<body><h1>Available ROS Image Topics:</h1>");
  connection->write("<ul>");
  BOOST_FOREACH(std::string& camera_info_topic, camera_info_topics){
    if(boost::algorithm::ends_with(camera_info_topic, "/camera_info")){
	std::string base_topic = camera_info_topic.substr(0, camera_info_topic.size()-strlen("camera_info"));
	connection->write("<li>");
	connection->write(base_topic);
	connection->write("<ul>");
	std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
	for ( ; image_topic_itr != image_topics.end(); ) {
	  if (boost::starts_with(*image_topic_itr, base_topic)) {
	    connection->write("<li><a href=\"/stream_viewer?topic=");
	    connection->write(*image_topic_itr);
	    connection->write("\">");
	    connection->write(image_topic_itr->substr(base_topic.size()));
	    connection->write("</a> (");
	    connection->write("<a href=\"/snapshot?topic=");
	    connection->write(*image_topic_itr);
	    connection->write("\">Snapshot</a>)");
	    connection->write("</li>");

	    image_topic_itr = image_topics.erase(image_topic_itr);
	  } else {
	    ++image_topic_itr;
	  }
	}
	connection->write("</ul>");
    }
    connection->write("</li>");
  }
  connection->write("</ul></body></html>");
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "web_video_server");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  web_video_server::WebVideoServer server(nh, private_nh);
  server.spin();

  return (0);
}

