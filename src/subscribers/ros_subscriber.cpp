#include "web_video_server/subscribers/ros_subscriber.hpp"

namespace web_video_server
{

RosSubscriber::RosSubscriber(rclcpp::Node::SharedPtr node)
{
  node_ = node;
}

RosSubscriber::~RosSubscriber()
{
}

void RosSubscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                              const std::string& topic,
                              const ImageCallback& callback)
{
  std::scoped_lock lock(subscriber_mutex_);
  
  callback_ = callback;
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    node_->get_logger(), "Streaming topic %s with QoS profile %s", topic.c_str(),
    qos_profile_name_.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name_);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name_.c_str());
  }

  rclcpp::QoS qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.value().history, 1),
    qos_profile.value());

  ros_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(topic, qos, std::bind(&RosSubscriber::subscriberCallback, this, std::placeholders::_1));
}


void RosSubscriber::subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr &input_msg)
{
  std::scoped_lock lock(subscriber_mutex_);
  callback_(input_msg);
}

}