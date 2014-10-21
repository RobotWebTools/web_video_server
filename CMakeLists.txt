cmake_minimum_required(VERSION 2.8.3)
project(web_video_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib cv_bridge image_transport)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

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
  src/jpeg_streamers.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} web_video_http_server
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
