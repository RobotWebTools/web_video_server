
#pragma once

#include <mutex>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <async_web_server_cpp/http_connection.hpp>

#include "web_video_server/utils.hpp"

namespace web_video_server
{
  typedef std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr&)> ImageCallback;

class RosSubscriber
{
  public:

    RosSubscriber(rclcpp::Node::SharedPtr node); 

    ~RosSubscriber();

    virtual void subscribe(const async_web_server_cpp::HttpRequest &request,
                           const std::string& topic,
                           const ImageCallback& callback);

    virtual void subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr &input_msg);

    std::mutex subscriber_mutex_;

  protected:
    rclcpp::Node::SharedPtr node_;
    std::string qos_profile_name_;

    ImageCallback callback_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_sub_;
};

class SubscriberType
{
  public:
    virtual std::shared_ptr<RosSubscriber> create_subscriber(rclcpp::Node::SharedPtr node) = 0;
};

} // web_video_server
