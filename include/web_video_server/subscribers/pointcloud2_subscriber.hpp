// Copyright (c) 2024-2025, The Robot Web Tools Contributors
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

#pragma once

#include <memory>
#include <string>

#include "async_web_server_cpp/http_request.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/logger.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

// #include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include "web_video_server/subscriber.hpp"

namespace web_video_server
{
namespace subscribers
{

class PointCloud2Subscriber : public SubscriberBase
{    
  public:
    PointCloud2Subscriber(rclcpp::Node::SharedPtr node);

    ~PointCloud2Subscriber();

    void subscribe(const async_web_server_cpp::HttpRequest &request,
                   const std::string& topic, 
                   const ImageCallback& callback);    
    

  private:
    void subscriberCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg);

    static bool compareFieldsOffset(sensor_msgs::msg::PointField& field1, sensor_msgs::msg::PointField& field2);
    static inline int sizeOfPointField(int datatype);    
    bool FindFields(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg, sensor_msgs::msg::PointField &userField, 
            sensor_msgs::msg::PointField &xField, sensor_msgs::msg::PointField &yField, sensor_msgs::msg::PointField &zField);
    sensor_msgs::msg::PointCloud2 TransformFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg, std::string frame_id);
    void GatherCameraInfo(cv::Mat &intrinsic_matrix, cv::Mat &distortion_coefficients);           
    bool CreateUserImage(const std_msgs::msg::Header &cloud_header, const sensor_msgs::msg::PointField &userField, cv_bridge::CvImage &userImage);
    bool CreateDepthImage(const std_msgs::msg::Header &cloud_header, cv_bridge::CvImage& depthImage);
    std::vector<cv::Point2f> ProjectPoints(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::PointCloud2 &output_cloud, 
            const sensor_msgs::msg::PointField &xField, const sensor_msgs::msg::PointField &yField, const sensor_msgs::msg::PointField &zField, 
            const cv::Mat &intrinsic_matrix, const cv::Mat &distortion_coefficients, std::vector<cv::Point3f> &obj_pts);
    cv_bridge::CvImage NormalizeImage(const cv_bridge::CvImage &inputImage);
    cv_bridge::CvImage ConvertToColor(const cv::Mat &depthMask, const cv_bridge::CvImage &inputImage);            

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    std::unique_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;    

    geometry_msgs::msg::TransformStamped transform_optical_;    

    std::string frame_id_;
    double wait_for_tf_delay_;
    std::string field_;
    int height_, width_, pixel_size_;
    double focal_length_;
    bool normalize_;
    bool colorize_;
	  rclcpp::CallbackGroup::SharedPtr cbg_;    
};

class PointCloud2SubscriberFactory : public SubscriberFactoryInterface
{
  public:
    std::string get_type() {return "sensor_msgs/msg/PointCloud2";}
    std::shared_ptr<SubscriberInterface> create_subscriber(
        rclcpp::Node::SharedPtr node);

    std::vector<std::string> get_available_topics(rclcpp::Node & node);
};

}  // namespace streamers
}  // namespace web_video_server