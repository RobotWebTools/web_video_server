# How to write a custom streamer plugin

This tutorial will guide you through the steps to create a simple custom streamer plugin for the `web_video_server` package in ROS 2. The example plugin will log messages when it is created, started, and when frames are restreamed.

1. Create you local workspace if you don't have one:
    ```bash
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/src
    ```
1. Create a new package for your custom streamer plugin:
    ```bash
    ros2 pkg create --build-type ament_cmake test_streamer_plugin --dependencies web_video_server pluginlib --library-name test_streamer_plugin
    cd test_streamer_plugin
    ```

1. Add `TestStreamer` and `TestStreamerFactory` classes to `include/test_streamer_plugin/test_streamer_plugin.hpp` header file:
    ```cpp
    #ifndef TEST_STREAMER_PLUGIN__TEST_STREAMER_PLUGIN_HPP_
    #define TEST_STREAMER_PLUGIN__TEST_STREAMER_PLUGIN_HPP_

    #include "test_streamer_plugin/visibility_control.h"

    #include "web_video_server/streamer.hpp"

    namespace test_streamer_plugin
    {

    class TestStreamer : public web_video_server::StreamerBase
    {
    public:
      TestStreamer(
        const async_web_server_cpp::HttpRequest & request,
        async_web_server_cpp::HttpConnectionPtr connection,
        rclcpp::Node::WeakPtr node);

      virtual ~TestStreamer();

      void start() override;
      void restream_frame(std::chrono::duration<double> max_age) override;
    };

    class TestStreamerFactory : public web_video_server::StreamerFactoryInterface
    {
    public:
      std::string get_type() override {return "test";}

      std::shared_ptr<web_video_server::StreamerInterface> create_streamer(
        const async_web_server_cpp::HttpRequest & request,
        async_web_server_cpp::HttpConnectionPtr connection,
        rclcpp::Node::WeakPtr node) override;
    };

    }  // namespace test_streamer_plugin

    #endif  // TEST_STREAMER_PLUGIN__TEST_STREAMER_PLUGIN_HPP_
    ```

1. Implement the `TestStreamer` and `TestStreamerFactory` classes in `src/test_streamer_plugin.cpp`:
    ```cpp
    #include "test_streamer_plugin/test_streamer_plugin.hpp"

    namespace test_streamer_plugin
    {

    TestStreamer::TestStreamer(
      const async_web_server_cpp::HttpRequest & request,
      async_web_server_cpp::HttpConnectionPtr connection,
      rclcpp::Node::WeakPtr node)
    : web_video_server::StreamerBase(request, connection, node, "test_streamer")
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
    }

    void TestStreamer::restream_frame(std::chrono::duration<double> max_age)
    {
      RCLCPP_INFO(logger_, "TestStreamer restream_frame called for topic: %s", topic_.c_str());
    }

    std::shared_ptr<web_video_server::StreamerInterface> TestStreamerFactory::create_streamer(
      const async_web_server_cpp::HttpRequest & request,
      async_web_server_cpp::HttpConnectionPtr connection,
      rclcpp::Node::WeakPtr node)
    {
      return std::make_shared<TestStreamer>(request, connection, node);
    }

    }  // namespace test_streamer_plugin

    #include "pluginlib/class_list_macros.hpp"

    PLUGINLIB_EXPORT_CLASS(
      test_streamer_plugin::TestStreamerFactory,
      web_video_server::StreamerFactoryInterface)
    ```

1. Add `plugins.xml` file with plugin description:
    ```xml
    <library path="test_streamer_plugin">
      <class name="test_streamer_plugin/stream/test"
        type="test_streamer_plugin::TestStreamerFactory"
        base_class_type="web_video_server::StreamerFactoryInterface">
        <description>Test streamer implementation</description>
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
    colcon build --packages-select test_streamer_plugin
    source install/setup.bash
    ```

1. Run the `web_video_server` node and test your custom streamer plugin by accessing a topic with the `test` format:
    ```bash
    ros2 run web_video_server web_video_server
    ```
    Then open your web browser and navigate to:
    ```
    http://localhost:8080/stream?topic=/your_image_topic&format=test
    ```

## Implementation hints
- You can access query parameters from the HTTP request in your streamer constructor using `request.get_query_param_value_or_default` method.
- Use `logger_` member variable from the base `StreamerBase` class for logging.
- Inherit from `web_video_server::streamers::ImageTransportStreamerBase` instead of `web_video_server::StreamerBase` if you want to use image transport functionality without writing boilerplate code.
- By default, in the topic list view, your custom streamer will not be shown for any topic. To change it, overwrite `get_available_topics` method in your `StreamerFactory` class or inherit from `ImageTransportStreamerFactoryBase` to make your format available for all topics using `sensor_msgs/msg/Image` message type.
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
    Add `web_video_server::web_video_server_streamers` if you inherit from `ImageTransportStreamerBase`.

