#include "web_video_server/subscribers/image_transport_subscriber.hpp"

namespace web_video_server
{

ImageTransportSubscriber::ImageTransportSubscriber(rclcpp::Node::SharedPtr node)
: RosSubscriber(node)
{
}

ImageTransportSubscriber::~ImageTransportSubscriber()
{
    std::scoped_lock lock(subscriber_mutex_);
}

void ImageTransportSubscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                         const std::string& topic, 
                                         const ImageCallback& callback)
{
  std::scoped_lock lock(subscriber_mutex_);
  
  callback_ = callback;
  std::string transport = request.get_query_param_value_or_default("transport", "raw");
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

  const auto qos = qos_profile.value();
  
  ros_sub_ = image_transport::create_subscription(node_.get(), topic, 
             std::bind(&ImageTransportSubscriber::subscriberCallback, this, std::placeholders::_1), 
             transport, qos);
}

void ImageTransportSubscriber::subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr &input_msg)
{
  std::scoped_lock lock(subscriber_mutex_);
  
  callback_(input_msg);
}


std::shared_ptr<RosSubscriber> ImageTransportSubscriberType::create_subscriber(rclcpp::Node::SharedPtr node)
{
  return std::shared_ptr<RosSubscriber>(
      new ImageTransportSubscriber(node));
}


}