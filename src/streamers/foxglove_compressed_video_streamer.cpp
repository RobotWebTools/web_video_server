// Copyright (c) 2026, The Robot Web Tools Contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "web_video_server/streamers/foxglove_compressed_video_streamer.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/asio/buffer.hpp>
#include <boost/system/system_error.hpp>

#include "async_web_server_cpp/http_reply.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"

#include "web_video_server/utils.hpp"

namespace web_video_server
{
namespace streamers
{
namespace
{

using Message = foxglove_msgs::msg::CompressedVideo;
constexpr const char * kMessageType = "foxglove_msgs/msg/CompressedVideo";
constexpr std::size_t kFrameHeaderSize = 20;

rclcpp::QoS make_qos(
  const std::string & topic, const std::string & profile_name, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Streaming topic %s with QoS profile %s", topic.c_str(), profile_name.c_str());
  auto profile = get_qos_profile_from_name(profile_name);
  if (!profile) {
    RCLCPP_ERROR(logger, "Invalid QoS profile %s; using default", profile_name.c_str());
    profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  }
  auto qos = profile.value();
  qos.keep_last(1);
  return qos;
}

bool topic_has_type(rclcpp::Node & node, const std::string & topic, const std::string & type)
{
  const auto names_and_types = node.get_topic_names_and_types();
  const auto it = names_and_types.find(topic);
  if (it == names_and_types.end()) {return false;}
  return std::find(it->second.begin(), it->second.end(), type) != it->second.end();
}

std::vector<std::string> collect_topics(rclcpp::Node & node)
{
  std::vector<std::string> result;
  for (const auto & item : node.get_topic_names_and_types()) {
    if (std::find(item.second.begin(), item.second.end(), kMessageType) != item.second.end()) {
      result.push_back(item.first);
    }
  }
  return result;
}

AVCodecID codec_id(const std::string & format)
{
  if (format == "h264") {return AV_CODEC_ID_H264;}
  if (format == "h265") {return AV_CODEC_ID_HEVC;}
  if (format == "vp9") {return AV_CODEC_ID_VP9;}
  if (format == "av1") {return AV_CODEC_ID_AV1;}
  return AV_CODEC_ID_NONE;
}

uint8_t codec_number(const std::string & format)
{
  if (format == "h264") {return 1;}
  if (format == "h265") {return 2;}
  if (format == "vp9") {return 3;}
  if (format == "av1") {return 4;}
  return 0;
}

void put_u32_be(uint8_t * destination, uint32_t value)
{
  destination[0] = static_cast<uint8_t>((value >> 24) & 0xff);
  destination[1] = static_cast<uint8_t>((value >> 16) & 0xff);
  destination[2] = static_cast<uint8_t>((value >> 8) & 0xff);
  destination[3] = static_cast<uint8_t>(value & 0xff);
}

bool annex_b_key_frame(const std::vector<uint8_t> & data, bool hevc)
{
  for (std::size_t i = 0; i + 4 < data.size(); ++i) {
    std::size_t nal = 0;
    if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
      nal = i + 3;
    } else if (
      i + 4 < data.size() && data[i] == 0 && data[i + 1] == 0 &&
      data[i + 2] == 0 && data[i + 3] == 1)
    {
      nal = i + 4;
    } else {
      continue;
    }
    if (nal >= data.size()) {continue;}
    if (!hevc && (data[nal] & 0x1f) == 5) {return true;}
    if (hevc) {
      const uint8_t type = (data[nal] >> 1) & 0x3f;
      if (type >= 16 && type <= 21) {return true;}
    }
  }
  return false;
}

}  // namespace

FoxgloveCompressedVideoStreamer::FoxgloveCompressedVideoStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
: StreamerBase(request, connection, node, "foxglove_compressed_video_streamer")
{
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");
  send_http_header();
}

FoxgloveCompressedVideoStreamer::~FoxgloveCompressedVideoStreamer()
{
  inactive_ = true;
  const std::scoped_lock lock(send_mutex_);
  if (parser_ != nullptr) {av_parser_close(parser_);}
  if (codec_context_ != nullptr) {avcodec_free_context(&codec_context_);}
}

void FoxgloveCompressedVideoStreamer::send_http_header()
{
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
  .header("Connection", "close")
  .header("Server", "web_video_server")
  .header("Cache-Control", "no-cache, no-store, must-revalidate")
  .header("Pragma", "no-cache")
  .header("Content-type", "application/x-foxglove-compressed-video")
  .header("Access-Control-Allow-Origin", "*")
  .write(connection_);
}

void FoxgloveCompressedVideoStreamer::start()
{
  auto node = lock_node();
  if (!node) {inactive_ = true; return;}
  video_sub_ = node->create_subscription<Message>(
    topic_, make_qos(topic_, qos_profile_name_, logger_),
    std::bind(&FoxgloveCompressedVideoStreamer::video_callback, this, std::placeholders::_1));
}

void FoxgloveCompressedVideoStreamer::restream_frame(
  std::chrono::duration<double> /* max_age */)
{
  // Repeating an encoded delta frame is invalid, so video streams are never restreamed.
}

void FoxgloveCompressedVideoStreamer::reset_parser(const std::string & format)
{
  if (parser_ != nullptr) {av_parser_close(parser_); parser_ = nullptr;}
  if (codec_context_ != nullptr) {avcodec_free_context(&codec_context_);}
  codec_ = nullptr;
  parser_format_ = format;

  const AVCodecID id = codec_id(format);
  if (id == AV_CODEC_ID_NONE) {return;}
  codec_ = avcodec_find_decoder(id);
  if (codec_ == nullptr) {return;}
  codec_context_ = avcodec_alloc_context3(codec_);
  parser_ = av_parser_init(id);
}

bool FoxgloveCompressedVideoStreamer::is_key_frame(const Message & msg)
{
  if (msg.format == "h264" && annex_b_key_frame(msg.data, false)) {return true;}
  if (msg.format == "h265" && annex_b_key_frame(msg.data, true)) {return true;}

  if (parser_ == nullptr || codec_context_ == nullptr || msg.data.empty()) {return false;}
  uint8_t * parsed_data = nullptr;
  int parsed_size = 0;
  const int consumed = av_parser_parse2(
    parser_, codec_context_, &parsed_data, &parsed_size,
    msg.data.data(), static_cast<int>(msg.data.size()),
    AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
  if (consumed < 0) {return false;}
  return parser_->key_frame == 1 || parser_->pict_type == AV_PICTURE_TYPE_I;
}

void FoxgloveCompressedVideoStreamer::video_callback(Message::ConstSharedPtr msg)
{
  const std::scoped_lock lock(send_mutex_);
  if (msg->format != parser_format_) {reset_parser(msg->format);}
  if (codec_number(msg->format) == 0) {
    RCLCPP_WARN(logger_, "Unsupported Foxglove compressed video format: %s", msg->format.c_str());
    return;
  }

  // Always forward encoded messages. The browser must receive parameter-set-only
  // messages so it can cache SPS/PPS before the first decodable IDR. The FFmpeg
  // parser result remains a useful keyframe hint, but it must not gate delivery.
  const bool key_frame = is_key_frame(*msg);
  send_frame(std::move(msg), key_frame);
}

void FoxgloveCompressedVideoStreamer::send_frame(Message::ConstSharedPtr msg, bool key_frame)
{
  auto header = std::make_shared<std::array<uint8_t, kFrameHeaderSize>>();
  auto & h = *header;
  h[0] = 'F'; h[1] = 'G'; h[2] = 'V'; h[3] = '1';
  h[4] = 1;
  h[5] = codec_number(msg->format);
  h[6] = key_frame ? 1 : 0;
  h[7] = 0;
  put_u32_be(h.data() + 8, static_cast<uint32_t>(msg->timestamp.sec));
  put_u32_be(h.data() + 12, msg->timestamp.nanosec);
  put_u32_be(h.data() + 16, static_cast<uint32_t>(msg->data.size()));

  try {
    connection_->write(boost::asio::buffer(*header), header);
    connection_->write(boost::asio::buffer(msg->data), msg);
  } catch (const boost::system::system_error & e) {
    RCLCPP_DEBUG(logger_, "Client disconnected: %s", e.what());
    inactive_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Could not send Foxglove compressed video: %s", e.what());
    inactive_ = true;
  }
}

std::shared_ptr<StreamerInterface> FoxgloveCompressedVideoStreamerFactory::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::WeakPtr node)
{
  auto locked = node.lock();
  if (!locked) {return nullptr;}
  const auto topic = request.get_query_param_value_or_default("topic", "");
  if (!topic_has_type(*locked, topic, kMessageType)) {
    RCLCPP_WARN(locked->get_logger(), "%s is not %s", topic.c_str(), kMessageType);
    return nullptr;
  }
  return std::make_shared<FoxgloveCompressedVideoStreamer>(request, connection, node);
}

std::vector<std::string> FoxgloveCompressedVideoStreamerFactory::get_available_topics(
  rclcpp::Node & node)
{
  return collect_topics(node);
}

std::string FoxgloveCompressedVideoStreamerFactory::create_viewer(
  const async_web_server_cpp::HttpRequest & request)
{
  std::stringstream stream;
  stream << R"HTML(
<canvas id="foxglove-video" style="max-width:100%;height:auto;background:#000"></canvas>
<div id="foxglove-video-status">Connecting...</div>
<script>
(() => {
  const canvas = document.getElementById('foxglove-video');
  const status = document.getElementById('foxglove-video-status');
  const context = canvas.getContext('2d', {alpha: false});
  let decoder;
  let configuredCodec = '';
  let configuredCodecNumber = 0;
  let firstTimestampUs;
  let buffer = new Uint8Array(0);
  let cachedH264Sps;
  let cachedH264Pps;

  function hasAnnexBStartCode(payload) {
    for (let i = 0; i + 3 < payload.length; ++i) {
      if (payload[i] !== 0 || payload[i + 1] !== 0) continue;
      if (payload[i + 2] === 1 || (payload[i + 2] === 0 && payload[i + 3] === 1)) return true;
    }
    return false;
  }

  function avccToAnnexB(payload) {
    const parts = [];
    let total = 0;
    let offset = 0;
    while (offset + 4 <= payload.length) {
      const length = new DataView(
        payload.buffer, payload.byteOffset + offset, 4).getUint32(0, false);
      offset += 4;
      if (length === 0 || offset + length > payload.length) return null;
      const part = payload.subarray(offset, offset + length);
      parts.push(part);
      total += 4 + part.length;
      offset += length;
    }
    if (offset !== payload.length || parts.length === 0) return null;
    const annexB = new Uint8Array(total);
    offset = 0;
    for (const part of parts) {
      annexB.set([0, 0, 0, 1], offset);
      offset += 4;
      annexB.set(part, offset);
      offset += part.length;
    }
    return annexB;
  }

  function normalizePayload(codecNumber, payload) {
    if (codecNumber !== 1 || hasAnnexBStartCode(payload)) return payload;
    const converted = avccToAnnexB(payload);
    if (!converted) throw new Error('H.264 payload is neither Annex-B nor 4-byte-length AVCC');
    return converted;
  }

  function h264NalUnits(payload) {
    const units = [];
    let start = -1;
    for (let i = 0; i + 3 < payload.length; ++i) {
      let length = 0;
      if (payload[i] === 0 && payload[i + 1] === 0 && payload[i + 2] === 1) length = 3;
      else if (payload[i] === 0 && payload[i + 1] === 0 && payload[i + 2] === 0 && payload[i + 3] === 1) length = 4;
      if (length === 0) continue;
      if (start >= 0 && i > start) units.push(payload.subarray(start, i));
      start = i + length;
      i += length - 1;
    }
    if (start >= 0 && start < payload.length) units.push(payload.subarray(start));
    return units;
  }

  function h264IsKeyFrame(payload) {
    return h264NalUnits(payload).some(unit => unit.length > 0 && (unit[0] & 0x1f) === 5);
  }

  function cacheH264ParameterSets(payload) {
    let hasVcl = false;
    for (const unit of h264NalUnits(payload)) {
      if (unit.length === 0) continue;
      const type = unit[0] & 0x1f;
      if (type === 7) cachedH264Sps = unit.slice();
      else if (type === 8) cachedH264Pps = unit.slice();
      else if (type >= 1 && type <= 5) hasVcl = true;
    }
    return hasVcl;
  }

  function prependH264ParameterSets(payload) {
    if (!cachedH264Sps || !cachedH264Pps) return null;
    const prefixLength = 4 + cachedH264Sps.length + 4 + cachedH264Pps.length;
    const combined = new Uint8Array(prefixLength + payload.length);
    let offset = 0;
    combined.set([0, 0, 0, 1], offset);
    offset += 4;
    combined.set(cachedH264Sps, offset);
    offset += cachedH264Sps.length;
    combined.set([0, 0, 0, 1], offset);
    offset += 4;
    combined.set(cachedH264Pps, offset);
    offset += cachedH264Pps.length;
    combined.set(payload, offset);
    return combined;
  }

  function codecName(number, payload) {
    if (number === 1) {
      for (const unit of h264NalUnits(payload)) {
        if (unit.length >= 4 && (unit[0] & 0x1f) === 7) {
          return 'avc1.' + [unit[1], unit[2], unit[3]]
            .map(value => value.toString(16).padStart(2, '0')).join('').toUpperCase();
        }
      }
      return 'avc1.42E01E';
    }
    if (number === 2) return 'hvc1.1.6.L93.B0';
    if (number === 3) return 'vp09.00.10.08';
    if (number === 4) return 'av01.0.04M.08';
    throw new Error('Unknown codec number ' + number);
  }

  async function configure(codecNumber, payload, keyFrame) {
    if (decoder && configuredCodecNumber === codecNumber) return;
    if (!keyFrame) throw new Error('Waiting for a key frame before configuring the decoder');
    const codec = codecName(codecNumber, payload);
    if (decoder) decoder.close();
    const config = {codec, optimizeForLatency: true, hardwareAcceleration: 'prefer-hardware'};
    const support = await VideoDecoder.isConfigSupported(config);
    if (!support.supported) throw new Error('Browser does not support ' + codec);
    decoder = new VideoDecoder({
      output: frame => {
        if (canvas.width !== frame.displayWidth || canvas.height !== frame.displayHeight) {
          canvas.width = frame.displayWidth;
          canvas.height = frame.displayHeight;
        }
        context.drawImage(frame, 0, 0, canvas.width, canvas.height);
        frame.close();
        status.textContent = codec + ' - ' + canvas.width + 'x' + canvas.height;
      },
      error: error => { status.textContent = 'Decoder error: ' + error.message; }
    });
    decoder.configure(support.config);
    configuredCodec = codec;
    configuredCodecNumber = codecNumber;
  }

  function append(chunk) {
    const joined = new Uint8Array(buffer.length + chunk.length);
    joined.set(buffer);
    joined.set(chunk, buffer.length);
    buffer = joined;
  }

  async function consumeFrames() {
    while (buffer.length >= 20) {
      if (buffer[0] !== 70 || buffer[1] !== 71 || buffer[2] !== 86 || buffer[3] !== 49) {
        throw new Error('Invalid video stream framing');
      }
      const view = new DataView(buffer.buffer, buffer.byteOffset, buffer.byteLength);
      const codecNumber = buffer[5];
      const keyFrame = (buffer[6] & 1) !== 0;
      const seconds = view.getInt32(8, false);
      const nanoseconds = view.getUint32(12, false);
      const payloadLength = view.getUint32(16, false);
      if (buffer.length < 20 + payloadLength) return;

      let payload = buffer.slice(20, 20 + payloadLength);
      buffer = buffer.slice(20 + payloadLength);
      payload = normalizePayload(codecNumber, payload);
      // Preserve the server-side FFmpeg parser result. The NAL scan is supplemental.
      let effectiveKeyFrame = keyFrame;
      if (codecNumber === 1) {
        effectiveKeyFrame = keyFrame || h264IsKeyFrame(payload);
        const hasVcl = cacheH264ParameterSets(payload);

        // h264parse may emit SPS/PPS separately from the following IDR access unit.
        // Keep those small parameter sets and prepend them only when a keyframe needs
        // to initialize a newly connected WebCodecs decoder.
        if (effectiveKeyFrame) {
          const units = h264NalUnits(payload);
          const hasSps = units.some(unit => unit.length > 0 && (unit[0] & 0x1f) === 7);
          const hasPps = units.some(unit => unit.length > 0 && (unit[0] & 0x1f) === 8);
          if (!hasSps || !hasPps) {
            const completed = prependH264ParameterSets(payload);
            if (!completed) {
              status.textContent = 'Waiting for H.264 SPS/PPS and a key frame';
              continue;
            }
            payload = completed;
          }
        } else if (!hasVcl) {
          // A parameter-set-only message updates the cache but is not a video frame.
          continue;
        }
      }
      const timestampUs = seconds * 1000000 + Math.floor(nanoseconds / 1000);
      if (firstTimestampUs === undefined) firstTimestampUs = timestampUs;
      await configure(codecNumber, payload, effectiveKeyFrame);
      decoder.decode(new EncodedVideoChunk({
        type: effectiveKeyFrame ? 'key' : 'delta',
        timestamp: timestampUs - firstTimestampUs,
        data: payload
      }));
    }
  }

  async function run() {
    if (!('VideoDecoder' in window)) throw new Error('This browser does not provide WebCodecs VideoDecoder');
    const response = await fetch('/stream?)HTML";
  stream << request.query;
  stream << R"HTML(');
    if (!response.ok || !response.body) throw new Error('HTTP ' + response.status);
    const reader = response.body.getReader();
    while (true) {
      const result = await reader.read();
      if (result.done) break;
      append(result.value);
      await consumeFrames();
    }
  }

  run().catch(error => { status.textContent = error.message; console.error(error); });
})();
</script>
)HTML";
  return stream.str();
}

}  // namespace streamers
}  // namespace web_video_server

PLUGINLIB_EXPORT_CLASS(
  web_video_server::streamers::FoxgloveCompressedVideoStreamerFactory,
  web_video_server::StreamerFactoryInterface)
