#include "web_video_server/libav_streamer.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

static int ffmpeg_boost_mutex_lock_manager(void **mutex, enum AVLockOp op)
{
  if (NULL == mutex)
    return -1;

  switch (op)
  {
    case AV_LOCK_CREATE:
    {
      *mutex = NULL;
      boost::mutex *m = new boost::mutex();
      *mutex = static_cast<void *>(m);
      break;
    }
    case AV_LOCK_OBTAIN:
    {
      boost::mutex *m = static_cast<boost::mutex *>(*mutex);
      m->lock();
      break;
    }
    case AV_LOCK_RELEASE:
    {
      boost::mutex *m = static_cast<boost::mutex *>(*mutex);
      m->unlock();
      break;
    }
    case AV_LOCK_DESTROY:
    {
      boost::mutex *m = static_cast<boost::mutex *>(*mutex);
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

LibavStreamer::LibavStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh,
                             const std::string &format_name, const std::string &codec_name,
                             const std::string &content_type) :
    ImageTransportImageStreamer(request, connection, nh), output_format_(0), format_context_(0), codec_(0), codec_context_(0), video_stream_(
        0), frame_(0), picture_(0), tmp_picture_(0), sws_context_(0), first_image_timestamp_(0), format_name_(
        format_name), codec_name_(codec_name), content_type_(content_type)
{

  bitrate_ = request.get_query_param_value_or_default<int>("bitrate", 100000);
  qmin_ = request.get_query_param_value_or_default<int>("qmin", 10);
  qmax_ = request.get_query_param_value_or_default<int>("qmax", 42);
  gop_ = request.get_query_param_value_or_default<int>("gop", 250);

  av_lockmgr_register(&ffmpeg_boost_mutex_lock_manager);
  av_register_all();
}

LibavStreamer::~LibavStreamer()
{
  if (codec_context_)
    avcodec_close(codec_context_);
  if (frame_)
  {
#if (LIBAVCODEC_VERSION_MAJOR < 54)
    av_free(frame_);
    frame_ = NULL;
#else
    avcodec_free_frame(&frame_);
#endif
  }
  if (format_context_)
    avformat_free_context(format_context_);
  if (picture_)
  {
    avpicture_free(picture_);
    delete picture_;
    picture_ = NULL;
  }
  if (tmp_picture_)
  {
    delete tmp_picture_;
    tmp_picture_ = NULL;
  }
  if (sws_context_)
    sws_freeContext(sws_context_);
}

void LibavStreamer::initialize(const cv::Mat &img)
{
  // Load format
  format_context_ = avformat_alloc_context();
  if (!format_context_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error allocating ffmpeg format context");
  }
  output_format_ = av_guess_format(format_name_.c_str(), NULL, NULL);
  if (!output_format_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error looking up output format");
  }
  format_context_->oformat = output_format_;

  // Load codec
  if (codec_name_.empty()) // use default codec if none specified
    codec_ = avcodec_find_encoder(output_format_->video_codec);
  else
    codec_ = avcodec_find_encoder_by_name(codec_name_.c_str());
  if (!codec_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error looking up codec");
  }
  video_stream_ = avformat_new_stream(format_context_, codec_);
  if (!video_stream_)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
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

  video_stream_->time_base.num = 1;
  video_stream_->time_base.den = 1000;

  codec_context_->time_base.num = 1;
  codec_context_->time_base.den = 1;
  codec_context_->gop_size = gop_;
  codec_context_->pix_fmt = PIX_FMT_YUV420P;
  codec_context_->max_b_frames = 0;

  // Quality settings
  codec_context_->qmin = qmin_;
  codec_context_->qmax = qmax_;

  initializeEncoder();

  // Some formats want stream headers to be separate
  if (format_context_->oformat->flags & AVFMT_GLOBALHEADER)
    codec_context_->flags |= CODEC_FLAG_GLOBAL_HEADER;

  // Open Codec
  if (avcodec_open2(codec_context_, codec_, NULL) < 0)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Could not open video codec");
  }

  // Allocate frame buffers
  frame_ = avcodec_alloc_frame();
  tmp_picture_ = new AVPicture;
  picture_ = new AVPicture;
  int ret = avpicture_alloc(picture_, codec_context_->pix_fmt, output_width_, output_height_);
  if (ret < 0)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Could not allocate picture frame");
  }
  *((AVPicture *)frame_) = *picture_;

  output_format_->flags |= AVFMT_NOFILE;

  // Generate header
  std::vector<uint8_t> header_buffer;
  std::size_t header_size;
  uint8_t *header_raw_buffer;
  // define meta data
  av_dict_set(&format_context_->metadata, "author", "ROS web_video_server", 0);
  av_dict_set(&format_context_->metadata, "title", topic_.c_str(), 0);

  if (avio_open_dyn_buf(&format_context_->pb) >= 0)
  {
    if (avformat_write_header(format_context_, NULL) < 0)
    {
      async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                           connection_,
                                                                                                           NULL, NULL);
      throw std::runtime_error("Error openning dynamic buffer");
    }
    header_size = avio_close_dyn_buf(format_context_->pb, &header_raw_buffer);

    // copy header buffer to vector
    header_buffer.resize(header_size);
    memcpy(&header_buffer[0], header_raw_buffer, header_size);

    av_free(header_raw_buffer);
  }

  // Send response headers
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Expires", "0").header("Max-Age", "0").header("Trailer", "Expires").header(
      "Content-type", content_type_).header("Access-Control-Allow-Origin", "*").write(connection_);

  // Send video stream header
  connection_->write_and_clear(header_buffer);
}

void LibavStreamer::initializeEncoder()
{
}

void LibavStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  boost::mutex::scoped_lock lock(encode_mutex_);
  if (first_image_timestamp_.isZero())
  {
    first_image_timestamp_ = time;
  }
  std::vector<uint8_t> encoded_frame;
#if (LIBAVUTIL_VERSION_MAJOR < 52)
  PixelFormat input_coding_format = PIX_FMT_BGR24;
#else
  AVPixelFormat input_coding_format = PIX_FMT_BGR24;
#endif
  avpicture_fill(tmp_picture_, img.data, input_coding_format, output_width_, output_height_);

  // Convert from opencv to libav
  if (!sws_context_)
  {
    static int sws_flags = SWS_BICUBIC;
    sws_context_ = sws_getContext(output_width_, output_height_, input_coding_format, output_width_, output_height_,
                                  codec_context_->pix_fmt, sws_flags, NULL, NULL, NULL);
    if (!sws_context_)
    {
      throw std::runtime_error("Could not initialize the conversion context");
    }
  }

  int ret = sws_scale(sws_context_, (const uint8_t * const *)tmp_picture_->data, tmp_picture_->linesize, 0,
                      output_height_, picture_->data, picture_->linesize);

  // Encode the frame
  AVPacket pkt;
  int got_packet;
  av_init_packet(&pkt);

#if (LIBAVCODEC_VERSION_MAJOR < 54)
  int buf_size = 6 * output_width_ * output_height_;
  pkt.data = (uint8_t*)av_malloc(buf_size);
  pkt.size = avcodec_encode_video(codec_context_, pkt.data, buf_size, frame_);
  got_packet = pkt.size > 0;
#else
  pkt.data = NULL; // packet data will be allocated by the encoder
  pkt.size = 0;
  if (avcodec_encode_video2(codec_context_, &pkt, frame_, &got_packet) < 0)
  {
    throw std::runtime_error("Error encoding video frame");
  }
#endif

  if (got_packet)
  {
    std::size_t size;
    uint8_t *output_buf;

    double seconds = (time - first_image_timestamp_).toSec();
    // Encode video at 1/0.95 to minimize delay
    pkt.pts = (int64_t)(seconds / av_q2d(video_stream_->time_base) * 0.95);
    if (pkt.pts <= 0)
      pkt.pts = 1;
    pkt.dts = AV_NOPTS_VALUE;

    if (codec_context_->coded_frame->key_frame)
      pkt.flags |= AV_PKT_FLAG_KEY;

    pkt.stream_index = video_stream_->index;

    if (avio_open_dyn_buf(&format_context_->pb) >= 0)
    {
      if (av_write_frame(format_context_, &pkt))
      {
        throw std::runtime_error("Error when writing frame");
      }
      size = avio_close_dyn_buf(format_context_->pb, &output_buf);

      encoded_frame.resize(size);
      memcpy(&encoded_frame[0], output_buf, size);

      av_free(output_buf);
    }
  }
  else
  {
    encoded_frame.clear();
  }
#if (LIBAVCODEC_VERSION_MAJOR < 54)
  av_free(pkt.data);
#endif

  av_free_packet(&pkt);

  connection_->write_and_clear(encoded_frame);
}

LibavStreamerType::LibavStreamerType(const std::string &format_name, const std::string &codec_name,
                                     const std::string &content_type) :
    format_name_(format_name), codec_name_(codec_name), content_type_(content_type)
{
}

boost::shared_ptr<ImageStreamer> LibavStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                                    async_web_server_cpp::HttpConnectionPtr connection,
                                                                    ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(
      new LibavStreamer(request, connection, nh, format_name_, codec_name_, content_type_));
}

std::string LibavStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<video src=\"/stream?";
  ss << request.query;
  ss << "\" autoplay=\"true\" preload=\"none\"></video>";
  return ss.str();
}

}
