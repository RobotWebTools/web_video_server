#include "web_video_server/libav_streamer.h"
#include "async_web_server_cpp/http_reply.hpp"

extern "C" {
#include "libavutil/imgutils.h" // av_image_get_buffer_size
}

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
    ImageTransportImageStreamer(request, connection, nh), output_format_(0), format_context_(0), codec_(0), codec_context_(
        0), video_stream_(0), frame_(0), bgr_frame_(0), sws_context_(0), first_image_timestamp_(0), format_name_(
        format_name), codec_name_(codec_name), content_type_(content_type), opt_(0), io_buffer_(0)
{
  // av_log_set_level(AV_LOG_DEBUG);
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
    av_frame_free(&frame_);
#endif
  }
  if (bgr_frame_)
  {
#if (LIBAVCODEC_VERSION_MAJOR < 54)
    av_free(bgr_frame_);
    bgr_frame_ = NULL;
#else
    av_frame_free(&bgr_frame_);
#endif
  }
  if (io_buffer_)
    delete io_buffer_;
  if (format_context_->pb)
    av_free(format_context_->pb);
  if (format_context_)
    avformat_free_context(format_context_);
  if (sws_context_)
    sws_freeContext(sws_context_);
}

// allocates and initializes an AVFrame
AVFrame* createVideoFrame(AVPixelFormat pix_fmt, int width, int height)
{
  AVFrame *frame = av_frame_alloc();
  if (!frame)
    return NULL;

  int size = av_image_get_buffer_size(pix_fmt, width, height, 16);
  uint8_t* buffer = (uint8_t*) av_malloc(size);
  if (!buffer)
  {
    av_frame_free(&frame);
    return NULL;
  }

  av_image_fill_arrays(frame->data, frame->linesize, buffer, pix_fmt, width, height, 1);

  frame->width = width;
  frame->height = height;
  frame->format = pix_fmt;

  return frame;
}

// output callback for ffmpeg IO context
static int dispatch_output_packet(void* opaque, uint8_t* buffer, int buffer_size)
{
  async_web_server_cpp::HttpConnectionPtr connection = *((async_web_server_cpp::HttpConnectionPtr*) opaque);
  std::vector<uint8_t> encoded_frame;
  encoded_frame.assign(buffer, buffer + buffer_size);
  connection->write_and_clear(encoded_frame);
  return 0; // TODO: can this fail?
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

  // Set up custom IO callback.
  size_t io_buffer_size = 3 * 1024 * 1024;    // 3M seen elsewhere and adjudged good
  io_buffer_ = new unsigned char[io_buffer_size];
  AVIOContext* io_ctx = avio_alloc_context(io_buffer_, io_buffer_size, AVIO_FLAG_WRITE, &connection_, NULL, dispatch_output_packet, NULL);
  if (!io_ctx)
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error setting up IO context");
  }
  io_ctx->seekable = 0;                       // no seeking, it's a stream
  format_context_->pb = io_ctx;
  output_format_->flags |= AVFMT_FLAG_CUSTOM_IO;
  output_format_->flags |= AVFMT_NOFILE;

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

  codec_context_->codec_id = codec_->id;

  codec_context_->width = output_width_;
  codec_context_->height = output_height_;
  codec_context_->delay = 0;

  codec_context_->time_base.num = 1;
  codec_context_->time_base.den = 1000;
  video_stream_->time_base = codec_context_->time_base; // just to surpress a warning, will be overwritten by FFmpeg later

  codec_context_->pix_fmt = PIX_FMT_YUV420P;

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
  frame_ = createVideoFrame(codec_context_->pix_fmt, codec_context_->width, codec_context_->height);
  bgr_frame_ = createVideoFrame(PIX_FMT_BGR24, codec_context_->width, codec_context_->height);
  if ((!frame_) || (!bgr_frame_))
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Could not allocate frame buffers");
  }

  // Generate header
  std::vector<uint8_t> header_buffer;
  std::size_t header_size;
  uint8_t *header_raw_buffer;
  // define meta data
  av_dict_set(&format_context_->metadata, "author", "ROS web_video_server", 0);
  av_dict_set(&format_context_->metadata, "title", topic_.c_str(), 0);

  // Send response headers
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok).header("Connection", "close").header(
      "Server", "web_video_server").header("Cache-Control",
                                           "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0").header(
      "Pragma", "no-cache").header("Expires", "0").header("Max-Age", "0").header("Trailer", "Expires").header(
      "Content-type", content_type_).header("Access-Control-Allow-Origin", "*").write(connection_);

  // Send video stream header
  if (avformat_write_header(format_context_, &opt_) < 0) // changes video stream timebase
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::internal_server_error)(request_,
                                                                                                         connection_,
                                                                                                         NULL, NULL);
    throw std::runtime_error("Error writing format header");
  }
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

  avpicture_fill((AVPicture *) bgr_frame_, img.data, PIX_FMT_BGR24, output_width_, output_height_);

  // Convert from opencv to libav
  if (!sws_context_)
  {
    static int sws_flags = SWS_BICUBIC;
    sws_context_ = sws_getContext(output_width_, output_height_, PIX_FMT_BGR24, output_width_, output_height_,
                                  codec_context_->pix_fmt, sws_flags, NULL, NULL, NULL);
    if (!sws_context_)
    {
      throw std::runtime_error("Could not initialize the conversion context");
    }
  }

  int ret = sws_scale(sws_context_, (const uint8_t * const *)bgr_frame_->data, bgr_frame_->linesize, 0,
                      output_height_, frame_->data, frame_->linesize);

  // Compute the presentation timestamp
  double seconds = (time - first_image_timestamp_).toSec();
  int64_t pts = (int64_t)(seconds / av_q2d(codec_context_->time_base));
  if (pts <= 0)
    pts = 1;
  frame_->pts = pts;

  // Encode the frame to a video packet
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
    pkt.pts = pts;
    pkt.dts = AV_NOPTS_VALUE;
    pkt.duration = 0;

    if (codec_context_->coded_frame->key_frame)
      pkt.flags |= AV_PKT_FLAG_KEY;

    // codec generates different pts than the stream timebase relevant to consumer
    av_packet_rescale_ts(&pkt, codec_context_->time_base, video_stream_->time_base);
    pkt.stream_index = video_stream_->index;

    if (av_write_frame(format_context_, &pkt))
    {
      throw std::runtime_error("Error when writing frame");
    }
  }

#if (LIBAVCODEC_VERSION_MAJOR < 54)
  av_free(pkt.data);
#endif

  av_packet_unref(&pkt);
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
