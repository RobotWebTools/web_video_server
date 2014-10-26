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


#include "web_video_server/libav_streamer.h"
#include "web_video_server/http_server/http_reply.hpp"

namespace web_video_server
{


static int ffmpeg_boost_mutex_lock_manager(void** mutex, enum AVLockOp op) {
  if (NULL == mutex)
    return -1;

  switch (op) {
  case AV_LOCK_CREATE:
    {
      *mutex = NULL;
      boost::mutex * m = new boost::mutex();
      *mutex = static_cast<void*>(m);
      break;
    }
  case AV_LOCK_OBTAIN:
    {
      boost::mutex * m = static_cast<boost::mutex*>(*mutex);
      m->lock();
      break;
    }
  case AV_LOCK_RELEASE:
    {
      boost::mutex * m = static_cast<boost::mutex*>(*mutex);
      m->unlock();
      break;
    }
  case AV_LOCK_DESTROY:
    {
      boost::mutex * m = static_cast<boost::mutex*>(*mutex);
      m->lock();
      m->unlock();
      delete m;
      break;
    }
  default:
    break;
  }
  return 0;
}


LibavStreamer::LibavStreamer(const http_server::HttpRequest& request,
			       http_server::HttpConnectionPtr connection,
			     image_transport::ImageTransport it,
			     const std::string& format_name, const std::string& codec_name,
			     const std::string& content_type)
  : ImageStreamer(request, connection, it),
    output_format_(0),
    format_context_(0),
    codec_(0),
    codec_context_(0),
    video_stream_(0),
    frame_(0),
    picture_(0),
    tmp_picture_(0),
    sws_context_(0),
    first_image_timestamp_(0),
    format_name_(format_name),
    codec_name_(codec_name),
    content_type_(content_type) {

  bitrate_ = request.get_query_param_value_or_default<int>("bitrate", 100000);
  qmin_ = request.get_query_param_value_or_default<int>("qmin", 10);
  qmax_ = request.get_query_param_value_or_default<int>("qmax", 42);
  gop_ = request.get_query_param_value_or_default<int>("gop", 250);

  av_lockmgr_register(&ffmpeg_boost_mutex_lock_manager);
  av_register_all();
}

LibavStreamer::~LibavStreamer(){
  if(codec_context_)
    avcodec_close(codec_context_);
  if(frame_)
    avcodec_free_frame(&frame_);
  if(format_context_)
    avformat_free_context(format_context_);
  if (picture_){
    avpicture_free(picture_);
    delete picture_;
    picture_ = NULL;
  }
  if (tmp_picture_){
    delete tmp_picture_;
    tmp_picture_ = NULL;
  }
  if (sws_context_)
    sws_freeContext(sws_context_);
}

void LibavStreamer::initialize(const cv::Mat& img){
  // Load format
  format_context_ = avformat_alloc_context();
  if (!format_context_){
    http_server::HttpReply::stock_reply(http_server::HttpReply::internal_server_error)(request_, connection_);
    throw std::runtime_error("Error allocating ffmpeg format context");
  }
  output_format_ = av_guess_format(format_name_.c_str(), NULL, NULL);
  if (!output_format_){
    http_server::HttpReply::stock_reply(http_server::HttpReply::internal_server_error)(request_, connection_);
    throw std::runtime_error("Error looking up output format");
  }
  format_context_->oformat = output_format_;


  // Load codec
  if (codec_name_.empty()) // use default codec if none specified
    codec_ = avcodec_find_encoder(output_format_->video_codec);
  else
    codec_ = avcodec_find_encoder_by_name(codec_name_.c_str());
  if (!codec_){
    http_server::HttpReply::stock_reply(http_server::HttpReply::internal_server_error)(request_, connection_);
    throw std::runtime_error("Error looking up codec");
  }
  video_stream_ = avformat_new_stream(format_context_, codec_);
  if (!video_stream_){
    http_server::HttpReply::stock_reply(http_server::HttpReply::internal_server_error)(request_, connection_);
    throw std::runtime_error("Error creating video stream");
  }
  codec_context_ = video_stream_->codec;

  // Set options
  avcodec_get_context_defaults3(codec_context_, codec_);

  codec_context_->codec_id = output_format_->video_codec;
  codec_context_->bit_rate = bitrate_;

  codec_context_->width = output_width_;
  codec_context_->height = output_height_;
  codec_context_->delay = 0;

  codec_context_->time_base.den = 1;
  codec_context_->time_base.num = 1;
  codec_context_->gop_size = gop_;
  codec_context_->pix_fmt = PIX_FMT_YUV420P;
  codec_context_->max_b_frames = 0;

  // Quality settings
  codec_context_->qmin = qmin_;
  codec_context_->qmax = qmax_;

  typedef std::map<std::string, std::string> AvOptMap;
  AvOptMap av_opt_map;
  av_opt_map["quality"] = "realtime";
  av_opt_map["deadline"] = "1";
  av_opt_map["auto-alt-ref"] = "0";
  av_opt_map["lag-in-frames"] = "1";
  av_opt_map["rc_lookahead"] = "1";
  av_opt_map["drop_frame"] = "1";
  av_opt_map["error-resilient"] = "1";

  for(AvOptMap::iterator itr = av_opt_map.begin(); itr != av_opt_map.end(); ++itr){
    av_opt_set(codec_context_->priv_data, itr->first.c_str(), itr->second.c_str(), 0);
  }

  // Buffering settings
  int bufsize = 10;
  codec_context_->rc_buffer_size = bufsize;
  codec_context_->rc_initial_buffer_occupancy = bufsize;//bitrate/3;
  av_opt_set_int(codec_context_->priv_data, "bufsize", bufsize, 0);
  av_opt_set_int(codec_context_->priv_data, "buf-initial", bufsize, 0);
  av_opt_set_int(codec_context_->priv_data, "buf-optimal", bufsize, 0);
  codec_context_->rc_buffer_aggressivity = 0.5;
  codec_context_->frame_skip_threshold = 10;

  // Some formats want stream headers to be separate
  if (format_context_->oformat->flags & AVFMT_GLOBALHEADER)
    codec_context_->flags |= CODEC_FLAG_GLOBAL_HEADER;


  // Open Codec
  if (avcodec_open2(codec_context_, codec_, NULL) < 0) {
    http_server::HttpReply::stock_reply(http_server::HttpReply::internal_server_error)(request_, connection_);
    throw std::runtime_error("Could not open video codec");
  }


  // Allocate frame buffers
  frame_ = avcodec_alloc_frame();
  tmp_picture_ = new AVPicture;
  picture_ = new AVPicture;
  int ret = avpicture_alloc(picture_, codec_context_->pix_fmt, output_width_, output_height_);
  if (ret < 0) {
    http_server::HttpReply::stock_reply(http_server::HttpReply::internal_server_error)(request_, connection_);
    throw std::runtime_error("Could not allocate picture frame");
  }
  *((AVPicture *)frame_) = *picture_;


  output_format_->flags |= AVFMT_NOFILE;


  // Generate header
  std::vector<uint8_t> header_buffer;
  std::size_t header_size;
  uint8_t* header_raw_buffer;
  // define meta data
  av_dict_set(&format_context_->metadata, "author", "ROS web_video_server", 0);
  av_dict_set(&format_context_->metadata, "title", topic_.c_str() , 0);

  if (avio_open_dyn_buf(&format_context_->pb) >= 0) {
    if (avformat_write_header(format_context_, NULL) < 0) {
      http_server::HttpReply::stock_reply(http_server::HttpReply::internal_server_error)(request_, connection_);
      throw std::runtime_error("Error openning dynamic buffer");
    }
    header_size = avio_close_dyn_buf(format_context_->pb, &header_raw_buffer);

    // copy header buffer to vector
    header_buffer.resize(header_size);
    memcpy(&header_buffer[0], header_raw_buffer, header_size);

    av_free(header_raw_buffer);
  }


  // Send response headers
  http_server::HttpReply::builder(http_server::HttpReply::ok)
    .header("Connection", "close")
    .header("Server", "web_video_server")
    .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
    .header("Pragma", "no-cache")
    .header("Expires", "0")
    .header("Max-Age", "0")
    .header("Trailer", "Expires")
    .header("Content-type", content_type_)
    .header("Access-Control-Allow-Origin", "*")
    .write(connection_);

  // Send video stream header
  connection_->write_and_clear(header_buffer);
}

void LibavStreamer::sendImage(const cv::Mat& img, const ros::Time& time) {
  boost::mutex::scoped_lock lock(encode_mutex_);
  if(first_image_timestamp_.isZero()) {
    first_image_timestamp_ = time;
  }
  std::vector<uint8_t> encoded_frame;
  AVPixelFormat input_coding_format = PIX_FMT_BGR24;
  avpicture_fill(tmp_picture_, img.data, input_coding_format, output_width_, output_height_);

  // Convert from opencv to libav
  if (!sws_context_) {
    static int sws_flags = SWS_BICUBIC;
    sws_context_ = sws_getContext(output_width_, output_height_, input_coding_format,
				  output_width_, output_height_, codec_context_->pix_fmt,
				  sws_flags, NULL, NULL, NULL);
    if (!sws_context_) {
      throw std::runtime_error("Could not initialize the conversion context");
    }
  }

  int ret = sws_scale(sws_context_,
		      (const uint8_t * const *)tmp_picture_->data,
		      tmp_picture_->linesize, 0, output_height_,
		      picture_->data, picture_->linesize);


  // Encode the frame
  AVPacket pkt;
  int got_packet;
  av_init_packet(&pkt);
  pkt.data = NULL; // packet data will be allocated by the encoder
  pkt.size = 0;

  if (avcodec_encode_video2(codec_context_, &pkt, frame_, &got_packet) < 0) {
    throw std::runtime_error("Error encoding video frame");
  }

  if (got_packet) {
    std::size_t size;
    uint8_t* output_buf;

    double seconds = (time - first_image_timestamp_).toSec();
    // Encode video at 1/0.95 to minimize delay
    pkt.pts = (int)(seconds / av_q2d(video_stream_->time_base) * 0.95);
    if(pkt.pts <= 0)
      pkt.pts = 1;

    if (codec_context_->coded_frame->key_frame)
      pkt.flags |= AV_PKT_FLAG_KEY;

    pkt.stream_index = video_stream_->index;

    if (avio_open_dyn_buf(&format_context_->pb) >= 0) {
      if (av_write_frame(format_context_, &pkt)) {
	throw std::runtime_error("Error when writing frame");
      }
      size = avio_close_dyn_buf(format_context_->pb, &output_buf);

      encoded_frame.resize(size);
      memcpy(&encoded_frame[0], output_buf, size);

      av_free(output_buf);
    }
  }
  else {
    encoded_frame.clear();
  }

  av_free_packet(&pkt);

  connection_->write_and_clear(encoded_frame);
}

LibavStreamerType::LibavStreamerType(const std::string& format_name, const std::string& codec_name,
				     const std::string& content_type)
  : format_name_(format_name), codec_name_(codec_name), content_type_(content_type) {}

boost::shared_ptr<ImageStreamer> LibavStreamerType::create_streamer(const http_server::HttpRequest& request,
								    http_server::HttpConnectionPtr connection,
								    image_transport::ImageTransport it) {
  return boost::shared_ptr<ImageStreamer>(new LibavStreamer(request, connection, it,
							    format_name_, codec_name_, content_type_));
}
std::string LibavStreamerType::create_viewer(const http_server::HttpRequest& request) {
  std::stringstream ss;
  ss << "<video src=\"/stream?";
  ss << request.query;
  ss << "\" autoplay=\"true\" preload=\"none\"></video>";
  return ss.str();
}



}
