
#pragma once

#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>

#include <web_video_server/subscribers/ros_subscriber.hpp>

namespace web_video_server
{

class ImageTransportSubscriber : public RosSubscriber
{    
  public:
    ImageTransportSubscriber(rclcpp::Node::SharedPtr node);

    ~ImageTransportSubscriber();

    virtual void subscribe(const async_web_server_cpp::HttpRequest &request,
                           const std::string& topic, 
                           const ImageCallback& callback);    
    
    void subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr &input_msg);
  
  private:
    image_transport::Subscriber ros_sub_;

};

class ImageTransportSubscriberType : public SubscriberType
{
  public:
    std::shared_ptr<RosSubscriber> create_subscriber(rclcpp::Node::SharedPtr node);
};

} //web_video_server
