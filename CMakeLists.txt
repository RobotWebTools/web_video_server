cmake_minimum_required(VERSION 3.5)
project(web_video_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(async_web_server_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(avformat libavformat REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

###################################################
## Declare things to be passed to other projects ##
###################################################

###########
## Build ##
###########

if(${cv_bridge_VERSION} VERSION_LESS "3.3.0")
  add_compile_definitions(CV_BRIDGE_USES_OLD_HEADERS)
endif()

## Specify additional locations of header files
include_directories(include
  ${avcodec_INCLUDE_DIRS}
  ${avformat_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  src/web_video_server.cpp
  src/image_streamer.cpp
  src/libav_streamer.cpp
  src/vp8_streamer.cpp
  src/h264_streamer.cpp
  src/vp9_streamer.cpp
  src/multipart_stream.cpp
  src/ros_compressed_streamer.cpp
  src/jpeg_streamers.cpp
  src/png_streamers.cpp
  src/utils.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  async_web_server_cpp::async_web_server_cpp
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  Boost::boost
  Boost::system
  ${OpenCV_LIBS}
  ${avcodec_LIBRARIES}
  ${avformat_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
  src/web_video_server_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "web_video_server::WebVideoServer")

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

###########
## Tests ##
###########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip ament_copyright check for humble
  if($ENV{ROS_DISTRO} STREQUAL "humble")
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_copyright
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
