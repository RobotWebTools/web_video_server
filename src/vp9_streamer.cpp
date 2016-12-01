#include "web_video_server/vp9_streamer.h"

namespace web_video_server
{

Vp9Streamer::Vp9Streamer(const async_web_server_cpp::HttpRequest& request,
                         async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
    LibavStreamer(request, connection, nh, "webm", "libvpx-vp9", "video/webm")
{
}
Vp9Streamer::~Vp9Streamer()
{
}

void Vp9Streamer::initializeEncoder()
{
  // codec options set up to provide somehow reasonable performance in cost of poor quality
  // should be updated as soon as VP9 encoding matures
  av_opt_set_int(codec_context_->priv_data, "pass", 1, 0);
  av_opt_set_int(codec_context_->priv_data, "speed", 8, 0);
  av_opt_set_int(codec_context_->priv_data, "cpu-used", 8, 0);  // 8 is max
  codec_context_->bit_rate = 0;                                 // variable bitrate
  av_opt_set_int(codec_context_->priv_data, "crf", 59, 0);      // 0..63 (higher is lower quality)
}

Vp9StreamerType::Vp9StreamerType() :
    LibavStreamerType("webm", "libvpx-vp9", "video/webm")
{
}

boost::shared_ptr<ImageStreamer> Vp9StreamerType::create_streamer(const async_web_server_cpp::HttpRequest& request,
                                                                  async_web_server_cpp::HttpConnectionPtr connection,
                                                                  ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new Vp9Streamer(request, connection, nh));
}

}
