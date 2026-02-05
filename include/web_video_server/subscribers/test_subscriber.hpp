#pragma once

#include "web_video_server/subscriber.hpp"

#include <opencv2/opencv.hpp>
#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

// replace the following line with one appropriate for you data type
#include <std_msgs/msg/string.hpp>

  namespace test_subscriber_plugin
  {

  class TestSubscriber : public web_video_server::SubscriberBase
  {
  public:
    TestSubscriber(rclcpp::Node::SharedPtr node);

    ~TestSubscriber();

    void subscribe(const async_web_server_cpp::HttpRequest &request,
                  const std::string& topic, 
                  const web_video_server::ImageCallback& callback);

  private:
    // replace param in the following line with one appropriate for you data type  
    void subscriberCallback(const std_msgs::msg::String::ConstSharedPtr &input_msg);

    // replace param in the following line with one appropriate for you data type  
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::CallbackGroup::SharedPtr cbg_;   
  };

  class TestSubscriberFactory : public web_video_server::SubscriberFactoryInterface
  {
  public:
    // replace the text string below with one appropriate for you data type
    // it should agree with value returned by the rclcpp function 
    // node.get_topic_names_and_types()
    std::string get_type() override {return "std_msgs/msg/String";}

    std::shared_ptr<web_video_server::SubscriberInterface> create_subscriber(
        rclcpp::Node::SharedPtr node);

    std::vector<std::string> get_available_topics(rclcpp::Node & node);
  };

  }  // namespace test_subscriber_plugin