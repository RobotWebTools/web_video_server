^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package web_video_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2019-09-20)
------------------
* Port to ROS 2

0.2.1 (2019-06-05)
------------------
* Restream buffered frames with minimum publish rate (`#88 <https://github.com/RobotWebTools/web_video_server/issues/88>`_)
  * Restream buffered frames with minimum publish rate
  * Implement restreaming for ros_compressed_streamer
* Update travis config (`#89 <https://github.com/RobotWebTools/web_video_server/issues/89>`_)
* Fall back to mjpeg if ros_compressed is unavailable (`#87 <https://github.com/RobotWebTools/web_video_server/issues/87>`_)
* Contributors: Jihoon Lee, Viktor Kunovski, sfalexrog

0.2.0 (2019-01-30)
------------------
* Add "default_stream_type" parameter (`#84 <https://github.com/RobotWebTools/web_video_server/issues/84>`_)
  This allows users to specify default stream type in their .launch files. Using a "ros_compressed" stream type sometimes
  results in a much lower resource consumption, and having it set as a default is much nicer for end users.
* Add a workaround for MultipartStream constant busy state (`#83 <https://github.com/RobotWebTools/web_video_server/issues/83>`_)
  * Add a workaround for MultipartStream constant busy state
  * Remove C++11 features
* lax rule for topic name (`#77 <https://github.com/RobotWebTools/web_video_server/issues/77>`_)
* Add PngStreamer (`#74 <https://github.com/RobotWebTools/web_video_server/issues/74>`_)
* fix SteadyTimer check for backported ROS versions (`#71 <https://github.com/RobotWebTools/web_video_server/issues/71>`_)
  i.e. on current kinetic
* Pkg format 2 (`#68 <https://github.com/RobotWebTools/web_video_server/issues/68>`_)
  * use package format 2
  * add missing dependency on sensor_msgs
* fixed undeclared CODEC_FLAG_GLOBAL_HEADER (`#65 <https://github.com/RobotWebTools/web_video_server/issues/65>`_)
* Contributors: Andreas Klintberg, Dirk Thomas, Felix Ruess, Kazuto Murase, Viktor Kunovski, sfalexrog

0.1.0 (2018-07-01)
------------------
* Avoid queuing of images on slow ethernet connection (`#64 <https://github.com/RobotWebTools/web_video_server/issues/64>`_)
* use SteadyTimer (if available) for cleaning up inactive streams (`#63 <https://github.com/RobotWebTools/web_video_server/issues/63>`_)
  * use SteadyTimer for cleaning up inactive streams
  so that cleanup works correctly even if system time changes
  SteadyTimer is available since roscpp 1.13.1
  * possibility to use SteadyTimer on older ROS versions
  when SteadyTimer has been backported to those...
* Fix segfault in libav_streamer destructor (resolves `#59 <https://github.com/RobotWebTools/web_video_server/issues/59>`_) (`#60 <https://github.com/RobotWebTools/web_video_server/issues/60>`_)
* Fix vp8 in kinetic add vp9 and h264 support (`#52 <https://github.com/RobotWebTools/web_video_server/issues/52>`_)
  * fix vp8 in kinetic
  * add h264 and vp9 support
* Add Indigo test matrix in travis configuration (`#50 <https://github.com/RobotWebTools/web_video_server/issues/50>`_)
* Set image streamer as inactive if topic is not available (`#53 <https://github.com/RobotWebTools/web_video_server/issues/53>`_)
  * Resolves `#38 <https://github.com/RobotWebTools/web_video_server/issues/38>`_
* Fix Build for ubuntu 14.04 (`#48 <https://github.com/RobotWebTools/web_video_server/issues/48>`_)
  * fix issue `#47 <https://github.com/RobotWebTools/web_video_server/issues/47>`_
  * fix double free
* Revert "use SteadyTimer for cleaning up inactive streams (`#45 <https://github.com/RobotWebTools/web_video_server/issues/45>`_)" (`#51 <https://github.com/RobotWebTools/web_video_server/issues/51>`_)
  This reverts commit ae74f19ada22f288a7c7a99ada7a1b9b6037c7ce.
* use SteadyTimer for cleaning up inactive streams (`#45 <https://github.com/RobotWebTools/web_video_server/issues/45>`_)
  so that cleanup works correctly even if system time changes
* Use trusty instead of xenial.  See `travis-ci/travis-ci#7260 <https://github.com/travis-ci/travis-ci/issues/7260>`_ (`#49 <https://github.com/RobotWebTools/web_video_server/issues/49>`_)
  * Also see `RobotWebTools/rosbridge_suite#311 <https://github.com/RobotWebTools/rosbridge_suite/issues/311>`_
* Contributors: Felix Ruess, James Bailey, Jihoon Lee, randoms, schallerr

0.0.7 (2017-11-20)
------------------
* Ffmpeg 3 (`#43 <https://github.com/RobotWebTools/web_video_server/issues/43>`_)
  * Correct use of deprecated parameters
  codec_context\_->rc_buffer_aggressivity marked as "currently useless", so removed
  codec_context\_->frame_skip_threshold access through new priv_data api
  * New names for pixel formats
  * AVPicture is deprecated, use AVFrame
  * Switch to non-deprecated free functions
  * Use new encoding api for newer versions
  * codec_context is deprecated, use packet flags
* Update travis configuration to test against kinetic (`#44 <https://github.com/RobotWebTools/web_video_server/issues/44>`_)
* fixed misuse of remove_if (`#35 <https://github.com/RobotWebTools/web_video_server/issues/35>`_)
* Merge pull request `#33 <https://github.com/RobotWebTools/web_video_server/issues/33>`_ from achim-k/patch-1
  web_video_server: fix bool function not returning
  This fix is required when compiling the package with `clang`. Otherwise a SIGILL (Illegal instruction) is triggered.
* Contributors: Hans-Joachim Krauch, Jan, Jihoon Lee, russelhowe

0.0.6 (2017-01-17)
------------------
* Fixed topic list to display all image topics, fixing Issue `#18 <https://github.com/RobotWebTools/web_video_server/issues/18>`_.
* Contributors: Eric

0.0.5 (2016-10-13)
------------------
* Merge pull request `#23 <https://github.com/RobotWebTools/web_video_server/issues/23>`_ from iki-wgt/develop
  More information when server creation is failed
* Removed empty line
* More detailed exception message
  Programm behavior is not changed since the exception is rethrown.
* Contributors: BennyRe, Russell Toris

0.0.4 (2015-08-18)
------------------
* Merge pull request #16 from mitchellwills/compressed
  Adds support for streaming ROS compressed image topics without the need to reencode them
* Switch to checkout async_web_server_cpp from source
* Upgrade for change in signature of async_web_server_cpp request handler
* Added ros compressed video streamer type
  This directly passes the ros compressed frame data to the http socket without reencoding it
* Switched from passing image transport to passing node handle to streamer constructors
* Added default transport parameter for regular image streamers
* Contributors: Mitchell Wills, Russell Toris

0.0.3 (2015-05-07)
------------------
* added verbose flag
* Contributors: Russell Toris

0.0.2 (2015-02-20)
------------------
* Merge pull request #10 from mitchellwills/develop
  Added option to specify server address
* Added option to specify server address
* Merge pull request #3 from mitchellwills/develop
  Remove old http_server implementation and replace it with async_web_server_cpp package
* Moved from using built in http server to new async_web_server_cpp package
* Did some cleanup of streamers
* Update package.xml
* Contributors: Mitchell Wills, Russell Toris

0.0.1 (2014-10-30)
------------------
* missed travis file
* cleanup and travis build
* ROS auto-format
* Merge pull request #1 from mitchellwills/develop
  Initial implementation of a http web server that serves ROS image topics as multiple web compatible formats
* Made some changes suggested by catkin_lint and did some package cleanup
* Added support for libavstreamer on Ubuntu 13.10 version of libav
* Added support for specifying vp8 quality parameter
* Implemented lazy initialization for libav buffers so that output size can be infered from the first image
* Updated README
* Added vp8 support
* Broke image encodings out into different files
* Made write operations async
  Send timestamps for mjpeg stream
* Initial commit
* Update README.md
* Update README.md
* Update README.md
* Initial commit
* Contributors: Mitchell Wills, Russell Toris
