# How to write a custom streamer plugin

Web_video_server contains two types of plugins: 1) subscribers 2) streamers.  Subscribers attach to ros topics and convert them into a standard image format.  Typically, streamers attach to subscribers, receive msg in the standard format, and publish the image data as web streams.  However, it is also possible for a streamer to bypass the subscriber pipeline and directly subscribe and process ros topics into streams.

This tutorial will guide you through the steps to create a simple custom streamer plugin for the `web_video_server` package in ROS 2. The example plugin will create ros log messages when it is created, started, and when new images are received from a subscriber or restreamed.

1. Create you local workspace if you don't have one:
  ```bash
  mkdir -p ~/ros_ws/src
  cd ~/ros_ws/src
  ```
2. Create a new package for your custom streamer plugin:
  ```bash
  ros2 pkg create --build-type ament_cmake test_streamer_plugin --dependencies web_video_server pluginlib --library-name test_streamer_plugin
  cd test_streamer_plugin
  ```
3. Add `TestStreamer` and `TestStreamerFactory` classes to `include/test_streamer_plugin/test_streamer_plugin.hpp` header file:
  ```cpp
  #pragma once

  #include "web_video_server/streamer.hpp"

  namespace test_streamer_plugin
  {

    class TestStreamer : public web_video_server::StreamerBase
    {
    public:
      TestStreamer(
        const async_web_server_cpp::HttpRequest & request,
        async_web_server_cpp::HttpConnectionPtr connection,
        std::map<std::string, std::shared_ptr<web_video_server::SubscriberFactoryInterface>> & subscriber_factories,  
        rclcpp::Node::WeakPtr node);

      virtual ~TestStreamer();

      void start();
      void restream_frame(std::chrono::duration<double> max_age);
      void subscriber_callback(const   sensor_msgs::msg::Image::ConstSharedPtr & msg);
    };

    class TestStreamerFactory : public web_video_server::StreamerFactoryInterface
    {
    public:
      std::string get_type() {return "test_streamer_plugin";}  
      std::vector<std::string> get_available_topics(
        rclcpp::Node & node,
        std::map<std::string, std::shared_ptr<web_video_server::SubscriberFactoryInterface>> subscriber_factories);  
      std::shared_ptr<web_video_server::StreamerInterface> create_streamer(
        const async_web_server_cpp::HttpRequest & request,
        async_web_server_cpp::HttpConnectionPtr connection,
        std::map<std::string, std::shared_ptr<web_video_server::SubscriberFactoryInterface>> & subscriber_factories,
        rclcpp::Node::WeakPtr node);
    };

  }  // namespace test_streamer_plugin
  ```
4. Implement the `TestStreamer` and `TestStreamerFactory` classes in `src/test_streamer_plugin.cpp`:

  ```cpp
  #include "test_streamer_plugin/test_streamer_plugin.hpp"

  namespace test_streamer_plugin
  {

    TestStreamer::TestStreamer(
      const async_web_server_cpp::HttpRequest & request,
      async_web_server_cpp::HttpConnectionPtr connection,
      std::map<std::string, std::shared_ptr<web_video_server::SubscriberFactoryInterface>> & subscriber_factories,  
      rclcpp::Node::WeakPtr node)
    : web_video_server::StreamerBase(request, connection, subscriber_factories, node, "test_streamer")
    {
      RCLCPP_INFO(logger_, "TestStreamer created for topic: %s", topic_.c_str());
    }

    TestStreamer::~TestStreamer()
    {
      RCLCPP_INFO(logger_, "TestStreamer destroyed for topic: %s", topic_.c_str());
    }

    void TestStreamer::start()
    {
      RCLCPP_INFO(logger_, "TestStreamer started for topic: %s", topic_.c_str());
      
      // Use subscriber factories to find an available subscriber for topic_ by msgs type. 
      // Attaches a callback that will trigger when new msgs is processed by subscriber.  
      // Remove this line and add your own pipeline to bypass subscriber functionality.
      attach_subscriber(std::bind(&TestStreamer::subscriber_callback, this, std::placeholders::_1));
    }

    void TestStreamer::restream_frame(std::chrono::duration<double> max_age)
    {
      RCLCPP_INFO(logger_, "TestStreamer restream_frame called for topic: %s", topic_.c_str());
    }

    void TestStreamer::subscriber_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
      RCLCPP_INFO(logger_, "TestStreamer image received from subscriber for topic: %s", topic_.c_str());
    }

    std::shared_ptr<web_video_server::StreamerInterface> TestStreamerFactory::create_streamer(
        const async_web_server_cpp::HttpRequest & request,
        async_web_server_cpp::HttpConnectionPtr connection,
        std::map<std::string, std::shared_ptr<web_video_server::SubscriberFactoryInterface>> & subscriber_factories,
        rclcpp::Node::WeakPtr node)
    {
      return std::make_shared<TestStreamer>(request, connection, subscriber_factories, node);
    }

    std::vector<std::string> TestStreamerFactory::get_available_topics(
      rclcpp::Node & node,
      std::map<std::string, std::shared_ptr<web_video_server::SubscriberFactoryInterface>> subscriber_factories)
    {
      std::vector<std::string> results;

      // Use subscriber factories to list avaiable topics by msgs type.
      // Remove this loop and add your own pipeline to bypass subscriber functionality.
      for (auto subscriber: subscriber_factories) {
        std::vector<std::string> entries = subscriber.second->get_available_topics(node);
        results.insert(results.end(), entries.begin(), entries.end());
      }

      return results;
    }

  }  // namespace test_streamer_plugin

  #include "pluginlib/class_list_macros.hpp"

  PLUGINLIB_EXPORT_CLASS(
    test_streamer_plugin::TestStreamerFactory,
    web_video_server::StreamerFactoryInterface)
  ```
5. Add `plugins.xml` file with plugin description:

  ```xml
  <library path="test_streamer_plugin">
    <class name="test_streamer_plugin/stream/test"
      type="test_streamer_plugin::TestStreamerFactory"
      base_class_type="web_video_server::StreamerFactoryInterface">
      <description>Test streamer implementation</description>
    </class>
  </library>
  ```
6. Update `CMakeLists.txt` to export the plugin description file (Add this anywhere after `find_package` section):

  ```cmake
  pluginlib_export_plugin_description_file(web_video_server plugins.xml)
  ```
7. Build your package:

  ```bash
  cd ~/ros_ws
  colcon build --packages-select test_streamer_plugin
  source install/setup.bash
  ```
8. Run the `web_video_server` node and test your custom streamer plugin by accessing a topic with the `test` format:

  ```bash
  ros2 run web_video_server web_video_server
  ```

  Then open your web browser and navigate to:

  ```
  http://localhost:8080/stream?topic=/your_image_topic&type=test_streamer_plugin
  ```

## Implementation hints

- You can access query parameters from the HTTP request in your streamer constructor using `request.get_query_param_value_or_default` method.
- Use `logger_` member variable from the base `StreamerBase` class for logging.
- Inherit from `web_video_server::streamers::ImageStreamerBase` instead of `web_video_server::StreamerBase` if you want access to default image functionality without writing boilerplate code (see jpeg_streamer for example).
- Link specific targets in `CMakeLists.txt`. For example, replace:

  ```cmake
  target_link_libraries(
    test_streamer_plugin PUBLIC
    ${web_video_server_TARGETS}
    ${pluginlib_TARGETS}
  )
  ```
  with:

  ```cmake
  target_link_libraries(
    test_streamer_plugin
    web_video_server::web_video_server
    pluginlib::pluginlib
  )
  ```
  Add `web_video_server::web_video_server_streamers` if you inherit from `ImageStreamerBase`.
