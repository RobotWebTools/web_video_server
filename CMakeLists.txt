cmake_minimum_required(VERSION 2.8.3)
project(web_video_server)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslib cv_bridge image_transport)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(avformat libavformat REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${avformat_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

add_library(web_video_http_server src/http_server/http_server.cpp
  src/http_server/http_connection.cpp src/http_server/http_request_parser.cpp
  src/http_server/http_reply.cpp src/http_server/request_handler.cpp
  src/http_server/http_request.cpp)
target_link_libraries(web_video_http_server
  ${Boost_LIBRARIES}
)


## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/web_video_server.cpp
  src/image_streamer.cpp
  src/libav_streamer.cpp
  src/jpeg_streamers.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${avcodec_LIBRARIES}
  ${avformat_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  web_video_http_server
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} web_video_http_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
