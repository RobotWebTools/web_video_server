
#pragma once

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "web_video_server/subscribers/ros_subscriber.hpp"

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif


namespace web_video_server
{

class PointCloud2Subscriber : public RosSubscriber
{    
  public:
    PointCloud2Subscriber(rclcpp::Node::SharedPtr node);

    ~PointCloud2Subscriber();

    virtual void subscribe(const async_web_server_cpp::HttpRequest &request,
                           const std::string& topic, 
                           const ImageCallback& callback);    
    
    void subscriberCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg);

    static bool compareFieldsOffset(sensor_msgs::msg::PointField& field1, sensor_msgs::msg::PointField& field2);
    static inline int sizeOfPointField(int datatype);
    
    bool FindFields(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg, sensor_msgs::msg::PointField &userField, 
            sensor_msgs::msg::PointField &xField, sensor_msgs::msg::PointField &yField, sensor_msgs::msg::PointField &zField);

    sensor_msgs::msg::PointCloud2 TransformFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg, std::string frame_id);
    void GatherCameraInfo(cv::Mat &intrinsic_matrix, cv::Mat &distortion_coefficients);           
    bool CreateUserImage(const std_msgs::msg::Header &cloud_header, const sensor_msgs::msg::PointField &userField, cv_bridge::CvImage &userImage);
    bool CreateDepthImage(const std_msgs::msg::Header &cloud_header, cv_bridge::CvImage& depthImage);
    std::vector<cv::Point2f> ProjectPoints(const sensor_msgs::msg::PointCloud2 &output_cloud, 
            const sensor_msgs::msg::PointField &xField, const sensor_msgs::msg::PointField &yField, const sensor_msgs::msg::PointField &zField, 
            const cv::Mat &intrinsic_matrix, const cv::Mat &distortion_coefficients, std::vector<cv::Point3f> &obj_pts);
    cv_bridge::CvImage NormalizeImage(const cv_bridge::CvImage &inputImage);
    cv_bridge::CvImage ConvertToColor(const cv::Mat &depthMask, const cv_bridge::CvImage &inputImage);

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ros_sub_;

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
	  rclcpp::CallbackGroup::SharedPtr cbg_sub_;    
};

class PointCloud2SubscriberType : public SubscriberType
{
  public:
    std::shared_ptr<RosSubscriber> create_subscriber(rclcpp::Node::SharedPtr node);
};

} //web_video_server
