# How to write a custom subscriber plugin

This tutorial will guide you through the steps to create a simple custom subscriber plugin for the `web_video_server` package in ROS 2. The example plugin will log messages when it is created, started, and when frames are restreamed.

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

    #include "web_video_server/subscriber.hpp"

    namespace test_subscriber_plugin
    {

    class TestSubscriber : public web_video_server::subscriberBase
    {
    public:
      TestSubscriber(rclcpp::Node::WeakPtr node);

      ~TestSubscriber();

      void subscribe(const async_web_server_cpp::HttpRequest &request,
                   const std::string& topic, 
                   const ImageCallback& callback);

    private:
      void subscriberCallback(const std_msgs::msg::String::ConstSharedPtr &input_msg);

      image_transport::Subscriber sub_;      
    };

    class TestSubscriberFactory : public web_video_server::SubscriberFactoryInterface
    {
    public:
      std::string get_type() override {return "std_msgs/msg/String";}

      std::shared_ptr<web_video_server::SubscriberInterface> create_subscriber(
          rclcpp::Node::WeakPtr node);

      std::vector<std::string> get_available_topics(rclcpp::Node & node);
    };

    }  // namespace test_subscriber_plugin

    ```

1. Implement the `TestSubscriber` and `TestSubscriberFactory` classes in `src/test_subscriber_plugin.cpp`:
    ```cpp
    #include "test_subscriber_plugin/test_subscriber_plugin.hpp"

    namespace test_subscriber_plugin
    {

    TestSubscriber::TestSubscriber(rclcpp::Node::WeakPtr _node)
    : web_video_server::SubscriberBase(node, "test_subscriber")
    {
      std::scoped_lock lock(subscriber_mutex_);
      
      RCLCPP_INFO(logger_, "TestSubscriber created for topic: %s", topic_.c_str());

      auto node = lock_node();
      if (!node) {
        return;
      }

      // Declare any new parameters required for this subscriber
      if (!node->has_parameter("test_parameter")) node->declare_parameter("test_parameter", "default");
    }

    TestSubscriber::~TestSubscriber()
    {
      std::scoped_lock lock(subscriber_mutex_);
      
      RCLCPP_INFO(logger_, "TestSubscriber destroyed for topic: %s", topic_.c_str());
    }

    void TestSubscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                   const std::string& topic, 
                                   const ImageCallback& callback)
    {
      std::scoped_lock lock(subscriber_mutex_);
      
      RCLCPP_INFO(logger_, "TestSubscriber started for topic: %s", topic_.c_str());

      // Load an parameters used by this subscriber
      std::string default_test_parameter = node->get_parameter("test_parameter").as_string();  
      std::string test_parameter = request.get_query_param_value_or_default("test_parameter", default_test_parameter);
      
      std::string default_qos_profile = node->get_parameter("default_qos_profile").as_string();    
      auto qos_profile_name = request.get_query_param_value_or_default("qos_profile", default_qos_profile);
      
      // Get QoS profile from query parameter
      RCLCPP_INFO(
        logger_, "Streaming topic %s with QoS profile %s", topic.c_str(),
        qos_profile_name.c_str());
      auto qos_profile = get_qos_profile_from_name(qos_profile_name);
      if (!qos_profile) {
        qos_profile = rmw_qos_profile_default;
        RCLCPP_ERROR(
          logger_,
          "Invalid QoS profile %s specified. Using default profile.",
          qos_profile_name.c_str());
      }
      const auto qos = qos_profile.value();
      
      // Create subscriber
      sub_ = node->create_subscription<std_msgs::msg::String>(topic, qos,
        std::bind(&TestSubscriber::subscriberCallback, this, std::placeholders::_1));
    }

    void SubscriberBase::subscriberCallback(const std_msgs::msg::Image::ConstSharedPtr &input_msg)
    {
      std::scoped_lock lock(subscriber_mutex_);
      
      // Convert input msg to image
      cv::Mat image(500, 500, CV_8UC3, Scalar(255, 255, 255));
      Point org(30, 100); // Top-left position of the text
      int fontFace = FONT_HERSHEY_SCRIPT_COMPLEX; // Font type
      double fontScale = 2.1; // Font scale
      Scalar color(0, 0, 255); // Text color (BGR format: 0, 0, 255 is red)
      int thickness = 2; // Text thickness
      int lineType = LINE_AA; // Anti-aliasing for smoother lines
      cv:putText(image, text, org, fontFace, fontScale, color, thickness, lineType);

      callback_(input_msg);
    }

    std::shared_ptr<web_video_server::subscriberInterface> TestSubscriberFactory::create_subscriber(
      const async_web_server_cpp::HttpRequest & request,
      async_web_server_cpp::HttpConnectionPtr connection,
      rclcpp::Node::WeakPtr node)
    {
      return std::make_shared<TestSubscriber>(request, connection, node);
    }

    std::vector<std::string> SubscriberFactoryInterface::get_available_topics(
      rclcpp::Node & /* node */
    ) {
      std::vector<std::string> result;
      auto topic_names_and_types = node.get_topic_names_and_types();
      for (const auto & topic_and_types : topic_names_and_types) {
        for (const auto & type : topic_and_types.second) {
          if (type == "std_msgs/msg/String") {
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
      web_video_server::subscriberFactoryInterface)
    ```

1. Add `plugins.xml` file with plugin description:
    ```xml
    <library path="test_subscriber_plugin">
      <class name="test_subscriber_plugin/stream/test"
        type="test_subscriber_plugin::TestSubscriberFactory"
        base_class_type="web_video_server::subscriberFactoryInterface">
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

1. Run the `web_video_server` node and test your custom subscriber plugin by accessing a topic with the `test` format:
    ```bash
    ros2 run web_video_server web_video_server
    ```
    Then open your web browser and navigate to:
    ```
    http://localhost:8080/stream?topic=/your_image_topic&format=test
    ```

## Implementation hints
- You can access query parameters from the HTTP request in your subscriber constructor using `request.get_query_param_value_or_default` method.
- Use `logger_` member variable from the base `subscriberBase` class for logging.
- Inherit from `web_video_server::subscribers::ImageTransportsubscriberBase` instead of `web_video_server::subscriberBase` if you want to use image transport functionality without writing boilerplate code.
- By default, in the topic list view, your custom subscriber will not be shown for any topic. To change it, overwrite `get_available_topics` method in your `subscriberFactory` class or inherit from `ImageTransportsubscriberFactoryBase` to make your format available for all topics using `std_msgs/msg/Image` message type.
- Link specific targets in `CMakeLists.txt`. For example, replace:
    ```cmake
    target_link_libraries(
      test_subscriber_plugin PUBLIC
      ${web_video_server_TARGETS}
      ${pluginlib_TARGETS}
    )
    ```
    with:
    ```cmake
    target_link_libraries(
      test_subscriber_plugin
      web_video_server::web_video_server
      pluginlib::pluginlib
    )
    ```
    Add `web_video_server::web_video_server_subscribers` if you inherit from `ImageTransportsubscriberBase`.

