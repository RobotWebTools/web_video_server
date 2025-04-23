# web_video_server - HTTP Streaming of ROS Image Topics in Multiple Formats

This node provides HTTP streaming of ROS image topics in various formats, making it easy to view robot camera feeds and other image topics in a web browser without requiring special plugins or extensions.

## Features

- Stream ROS image topics over HTTP in multiple formats:
  - MJPEG (Motion JPEG)
  - VP8 (WebM)
  - VP9 (WebM)
  - H264 (MP4)
  - PNG streams
  - ROS compressed image topics
- Adjustable quality, size, and other streaming parameters
- Web interface to browse available image topics
- Single image snapshot capability
- Support for different QoS profiles in ROS 2

## Installation

### Dependencies

- ROS (Noetic) or ROS 2 (Humble+)
- OpenCV
- FFmpeg/libav
- Boost
- async_web_server_cpp
 
### Installing packages

For newer ROS2 distributions (humble, jazzy, rolling) it is possible to install web_video_server as a package:

```
sudo apt install ros-${ROS_DISTRO}-web-video-server
```

### Building from Source

Create a ROS workspace if you don't have one:
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

Clone this repository:
```bash
# ROS 2
git clone https://github.com/RobotWebTools/web_video_server.git
# ROS 1
git clone https://github.com/RobotWebTools/web_video_server.git -b ros1
```

Install dependencies with rosdep:
```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -i
```

Build the package and source your workspace:
```bash
colcon build --packages-select web_video_server
source install/setup.bash
```

## Usage

### Starting the Server

```bash
# ROS 1
rosrun web_video_server web_video_server

# ROS 2
ros2 run web_video_server web_video_server
```


### Configuration

#### Server Configuration Parameters

| Parameter | Type | Default | Possible Values | Description |
|-----------|------|---------|----------------|-------------|
| `port` | int | 8080 | Any valid port number | HTTP server port |
| `address` | string | "0.0.0.0" | Any valid IP address | HTTP server address (0.0.0.0 allows external connections) |
| `server_threads` | int | 1 | 1+ | Number of server threads for handling HTTP requests |
| `ros_threads` | int | 2 | 1+ | Number of threads for ROS message handling |
| `verbose` | bool | false | true, false | Enable verbose logging |
| `default_stream_type` | string | "mjpeg" | "mjpeg", "vp8", "vp9", "h264", "png", "ros_compressed" | Default format for video streams |
| `publish_rate` | double | -1.0 | -1.0 or positive value | Rate for republishing images (-1.0 means no republishing) |

#### Running with Custom Parameters

You can configure the server by passing parameters via the command line:

```bash
# ROS 1
rosrun web_video_server web_video_server _port:=8081 _address:=localhost _server_threads:=4

# ROS 2
ros2 run web_video_server web_video_server --ros-args -p port:=8081 -p address:=localhost -p server_threads:=4
```

### View Available Streams
```
http://localhost:8080/
```
The interface allows quick navigation between different topics and formats without having to manually construct URLs.

This page displays:
- All available ROS image topics currently being published
- Direct links to view each topic in different formats:
  - Web page with streaming image
  - Direct stream
  - Single image snapshot

### Stream an Image Topic

There are two ways to stream the Image, as a HTML page via 
```
http://localhost:8080/stream_viewer?topic=/camera/image_raw
```
or as a HTTP multipart stream on

```
http://localhost:8080/stream?topic=/camera/image_raw
```
#### URL Parameters for Streaming

The following parameters can be added to the stream URL:

| Parameter | Type | Default | Possible Values | Description |
|-----------|------|---------|----------------|-------------|
| `topic` | string | (required) | Any valid ROS image topic | The ROS image topic to stream |
| `type` | string | "mjpeg" | "mjpeg", "vp8", "vp9", "h264", "png", "ros_compressed" | Stream format |
| `width` | int | 0 | 0+ | Width of output stream (0 = original width) |
| `height` | int | 0 | 0+ | Height of output stream (0 = original height) |
| `quality` | int | 95 | 1-100 | Quality for MJPEG and PNG streams |
| `bitrate` | int | 100000 | Positive integer | Bitrate for H264/VP8/VP9 streams in bits/second |
| `invert` | flag | not present | present/not present | Invert image when parameter is present |
| `default_transport` | string | "raw" | "raw", "compressed", "theora" | Image transport to use |
| `qos_profile` | string | "default" | "default", "system_default", "sensor_data", "services_default" | QoS profile for ROS 2 subscribers |

Examples:

```
# Stream an MJPEG at 640x480 with 90% quality
http://localhost:8080/stream?topic=/camera/image_raw&type=mjpeg&width=640&height=480&quality=90

# Stream H264 with higher bitrate
http://localhost:8080/stream?topic=/camera/image_raw&type=h264&bitrate=500000

# Stream with inverted image (rotated 180°)
http://localhost:8080/stream?topic=/camera/image_raw&invert

```

### Get a Snapshot
It is also possible to get a single image snapshot 
```
http://localhost:8080/snapshot?topic=/camera/image_raw
```
#### URL Parameters for Snapshot

| Parameter | Type | Default | Possible Values | Description |
|-----------|------|---------|----------------|-------------|
| `topic` | string | (required) | Any valid ROS image topic | The ROS image topic to stream |
| `width` | int | 0 | 0+ | Width of output picture (0 = original width) |
| `height` | int | 0 | 0+ | Height of output picture (0 = original height) |
| `quality` | int | 95 | 1-100 | Quality for JPEG snapshots |
| `invert` | flag | not present | present/not present | Invert image when parameter is present |
| `default_transport` | string | "raw" | "raw", "compressed", "theora" | Image transport to use |
| `qos_profile` | string | "default" | "default", "system_default", "sensor_data", "services_default" | QoS profile for ROS 2 subscribers |

## About

This project is released as part of the [Robot Web Tools](https://robotwebtools.github.io/) effort.

## License
web_video_server is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

## Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
