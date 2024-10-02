#ifndef LIBAV_STREAMERS_H_
#define LIBAV_STREAMERS_H_

#include <image_transport/image_transport.hpp>
#include "web_video_server/image_streamer.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/intreadwrite.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>
#include <libavutil/imgutils.h>
}

namespace web_video_server
{

class LibavStreamer : public ImageTransportImageStreamer
{
public:
  LibavStreamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node, const std::string & format_name, const std::string & codec_name,
    const std::string & content_type);

  ~LibavStreamer();

protected:
  virtual void initializeEncoder();
  virtual void sendImage(const cv::Mat &, const rclcpp::Time & time);
  virtual void initialize(const cv::Mat &);
  AVFormatContext * format_context_;
  const AVCodec * codec_;
  AVCodecContext * codec_context_;
  AVStream * video_stream_;

  AVDictionary * opt_;   // container format options

private:
  AVFrame * frame_;
  struct SwsContext * sws_context_;
  rclcpp::Time first_image_timestamp_;
  boost::mutex encode_mutex_;

  std::string format_name_;
  std::string codec_name_;
  std::string content_type_;
  int bitrate_;
  int qmin_;
  int qmax_;
  int gop_;

  uint8_t * io_buffer_;  // custom IO buffer
};

class LibavStreamerType : public ImageStreamerType
{
public:
  LibavStreamerType(
    const std::string & format_name, const std::string & codec_name,
    const std::string & content_type);

  boost::shared_ptr<ImageStreamer> create_streamer(
    const async_web_server_cpp::HttpRequest & request,
    async_web_server_cpp::HttpConnectionPtr connection,
    rclcpp::Node::SharedPtr node);

  std::string create_viewer(const async_web_server_cpp::HttpRequest & request);

private:
  const std::string format_name_;
  const std::string codec_name_;
  const std::string content_type_;
};

}

#endif
