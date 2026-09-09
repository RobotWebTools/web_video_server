# How to write a custom subscriber plugin

Web_video_server contains two types of plugins: 1) subscribers 2) streamers.  Subscribers attach to ros topics and convert them into a standard image format.  Streamers attach to subscribers, receive msg in the standard format, and publish the image data as web streams.

This tutorial will guide you through the steps to create a simple custom subscriber plugin for the `web_video_server` package in ROS 2. The example plugin will subscribe to a new msg type, covert it to a sensor_msgs/msg/Image and forward it to a user selected streamer.

1. Create you local workspace if you don't have one:
  ```bash
  mkdir -p ~/ros_ws/src
  cd ~/ros_ws/src
  ```
1. Create a new package for your custom subscriber plugin:
  ```bash
  ros2 pkg create --build-type ament_cmake test_subscriber_plugin --dependencies web_video_server pluginlib --library-name test_subscriber_plugin
  cd test_subscriber_plugin
  ```

1. Add `TestSubscriber` and `TestSubscriberFactory` classes to `include/test_subscriber_plugin/test_subscriber_plugin.hpp` header file:
  ```cpp
  #pragma once

  #include "web_video_server/utils.hpp"
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
      TestSubscriber(rclcpp::Node::WeakPtr node);

      ~TestSubscriber();

      void subscribe(const async_web_server_cpp::HttpRequest &request,
                    const std::string& topic, 
                    const web_video_server::ImageCallback& callback);

    private:
      void try_forward_image(const sensor_msgs::msg::Image::ConstSharedPtr & input_msg)
      {
        try {
          callback_(input_msg);
        } catch (...) {
          RCLCPP_ERROR(logger_, "The subscriber plugin failed send image for some reason.");
        }
      }

      // replace param in the following line with one appropriate for you data type  
      void subscriber_callback(const std_msgs::msg::String::ConstSharedPtr &input_msg);

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
  ```

1. Implement the `TestSubscriber` and `TestSubscriberFactory` classes in `src/test_subscriber_plugin.cpp`:
  ```cpp
  #include "test_subscriber_plugin/test_subscriber_plugin.hpp"

  namespace test_subscriber_plugin
  {

  TestSubscriber::TestSubscriber(rclcpp::Node::WeakPtr node)
  : web_video_server::SubscriberBase(node, "test_subscriber")
  {
    auto node_ptr = lock_node();
    if (!node_ptr) {
      inactive_ = true;
      return;
    }

    const std::scoped_lock lock(subscriber_mutex_);
    
    RCLCPP_INFO(logger_, "TestSubscriber created!");

    // Declare any new parameters required for this subscriber
    if (!node_ptr->has_parameter("test_parameter")) {
      node_ptr->declare_parameter("test_parameter", "default");
    }
  }

  TestSubscriber::~TestSubscriber()
  {
    const std::scoped_lock lock(subscriber_mutex_);
    inactive_ = true;
    
    RCLCPP_INFO(logger_, "TestSubscriber destroyed!");
  }

  void TestSubscriber::subscribe(
    const async_web_server_cpp::HttpRequest &request,
    const std::string& topic, 
    const web_video_server::ImageCallback& callback)
  {
    auto node = lock_node();
    if (!node) {
      inactive_ = true;
      return;
    }
    
    const std::scoped_lock lock(subscriber_mutex_);

    callback_ = callback;
    
    RCLCPP_INFO(logger_, "TestSubscriber started for topic: %s", topic.c_str());

    // Load parameters used by this subscriber
    const std::string default_test_parameter = node->get_parameter("test_parameter").as_string();  
    const std::string test_parameter = request.get_query_param_value_or_default(
      "test_parameter", 
      default_test_parameter);
    
    const std::string default_qos_profile = node->get_parameter("default_qos_profile").as_string();
    auto qos_profile_name = request.get_query_param_value_or_default(
      "qos_profile",
      default_qos_profile);

    // Get QoS profile from query parameter
    RCLCPP_INFO(
      logger_, "Streaming topic %s with QoS profile %s", topic.c_str(),
      qos_profile_name.c_str());
    auto qos_profile = web_video_server::get_qos_profile_from_name(qos_profile_name);
    if (!qos_profile) {
      qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
      RCLCPP_ERROR(
        logger_,
        "Invalid QoS profile %s specified. Using default profile.",
        qos_profile_name.c_str());
    }

    cbg_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);    
    rclcpp::SubscriptionOptions options;
    options.callback_group = cbg_;  
    
    // Create subscriber (update as appropriate for your subscriber)
    sub_ = node->create_subscription<std_msgs::msg::String>(
      topic, qos_profile.value(), 
      std::bind(&TestSubscriber::subscriber_callback, this, std::placeholders::_1), options
    );
  }

  void TestSubscriber::subscriber_callback(const std_msgs::msg::String::ConstSharedPtr &input_msg)
  {
    const std::scoped_lock lock(subscriber_mutex_);

    if(inactive_) return;      

    RCLCPP_INFO_STREAM(logger_,  "New TestSubscriber msg: " << input_msg->data);
    
    // Convert input msg to image
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    cv:putText(image, input_msg->data, cv::Point(30,250), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

    // Send to streamer using callback      
    cv_bridge::CvImage bridge_image(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, image);
    sensor_msgs::msg::Image output_msg;
    bridge_image.toImageMsg(output_msg);
    sensor_msgs::msg::Image::ConstSharedPtr output_ptr = std::make_shared<sensor_msgs::msg::Image>(output_msg);
    try_forward_image(output_ptr);
  }

  std::shared_ptr<web_video_server::SubscriberInterface> TestSubscriberFactory::create_subscriber(
    rclcpp::Node::SharedPtr node
  ) {
    return std::make_shared<TestSubscriber>(node);
  }

  std::vector<std::string> TestSubscriberFactory::get_available_topics(rclcpp::Node & node)
  {
    std::vector<std::string> result;
    auto topic_names_and_types = node.get_topic_names_and_types();
    for (const auto & topic_and_types : topic_names_and_types) {
      for (const auto & type : topic_and_types.second) {
        if (type == this->get_type()) {
          result.push_back(topic_and_types.first);
          break;
        }
      }
    }
    return result;
  }

  }  // namespace test_subscriber_plugin

  #include "pluginlib/class_list_macros.hpp"

  PLUGINLIB_EXPORT_CLASS(
    test_subscriber_plugin::TestSubscriberFactory,
    web_video_server::SubscriberFactoryInterface)
  ```

1. Add `plugins.xml` file with plugin description:
  ```xml
  <library path="test_subscriber_plugin">
    <class name="test_subscriber_plugin/subscriber/test"
      type="test_subscriber_plugin::TestSubscriberFactory"
      base_class_type="web_video_server::SubscriberFactoryInterface">
      <description>Test subscriber implementation</description>
    </class>
  </library>
  ```

1. Update `CMakeLists.txt` to export the plugin description file (Add this anywhere after `find_package` section):
  ```cmake
  pluginlib_export_plugin_description_file(web_video_server plugins.xml)
  ```

1. Build your package:
  ```bash
  cd ~/ros_ws
  colcon build --packages-select test_subscriber_plugin
  source install/setup.bash
  ```

1. Run the `web_video_server` node and test your custom subscriber plugin by accessing a topic of the appropriate format:
  ```bash
  ros2 topic pub /your_topic std_msgs/msg/String "data: test"
  ros2 run web_video_server web_video_server --ros-args -p port:=8082 -p address:=localhost
  ```
  Then open your web browser and navigate to:
  ```
  http://localhost:8082/stream?topic=/your_topic
  ```

## Implementation hints
- You can access query parameters from the HTTP request in your subscriber constructor using `request.get_query_param_value_or_default` method.
- Use `logger_` member variable from the base `SubscriberBase` class for logging.