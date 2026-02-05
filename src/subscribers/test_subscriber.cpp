#include "web_video_server/subscribers/test_subscriber.hpp"

    namespace test_subscriber_plugin
    {

    TestSubscriber::TestSubscriber(rclcpp::Node::SharedPtr node)
    : web_video_server::SubscriberBase(node, "test_subscriber")
    {
      std::scoped_lock lock(subscriber_mutex_);
      
      RCLCPP_INFO(logger_, "TestSubscriber created!");

      // Declare any new parameters required for this subscriber
      if (!node_->has_parameter("test_parameter")) node_->declare_parameter("test_parameter", "default");
    }

    TestSubscriber::~TestSubscriber()
    {
      std::scoped_lock lock(subscriber_mutex_);
      
      RCLCPP_INFO(logger_, "TestSubscriber destroyed!");
    }

    void TestSubscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                   const std::string& topic, 
                                   const web_video_server::ImageCallback& callback)
    {
      std::scoped_lock lock(subscriber_mutex_);

      callback_ = callback;
      
      RCLCPP_INFO(logger_, "TestSubscriber started for topic: %s", topic.c_str());

      // Load parameters used by this subscriber
      std::string default_test_parameter = node_->get_parameter("test_parameter").as_string();  
      std::string test_parameter = request.get_query_param_value_or_default("test_parameter", default_test_parameter);
      
      std::string default_qos_profile = node_->get_parameter("default_qos_profile").as_string();    
      auto qos_profile_name = request.get_query_param_value_or_default("qos_profile", default_qos_profile);
      
      // Get QoS profile from query parameter
      RCLCPP_INFO(
        logger_, "Streaming topic %s with QoS profile %s", topic.c_str(),
        qos_profile_name.c_str());
      auto qos_profile = web_video_server::get_qos_profile_from_name(qos_profile_name);
      if (!qos_profile) {
        qos_profile = rmw_qos_profile_default;
        RCLCPP_ERROR(
          logger_,
          "Invalid QoS profile %s specified. Using default profile.",
          qos_profile_name.c_str());
      }

      const auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.value().history, 1),
        qos_profile.value());

      cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);    
      rclcpp::SubscriptionOptions options;
      options.callback_group = cbg_;  
      
      // Create subscriber (update as appropriate for your subscriber)
      sub_ = node_->create_subscription<std_msgs::msg::String>(
        topic, qos, std::bind(&TestSubscriber::subscriberCallback, this, std::placeholders::_1), options
      );
    }

    void TestSubscriber::subscriberCallback(const std_msgs::msg::String::ConstSharedPtr &input_msg)
    {
      std::scoped_lock lock(subscriber_mutex_);

      RCLCPP_INFO_STREAM(logger_,  "New TestSubscriber msg: " << input_msg->data);
      
      // Convert input msg to image
      cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
      cv:putText(image, input_msg->data, cv::Point(30,250), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

      // Send to streamer using callback      
      cv_bridge::CvImage bridge_image(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, image);
      sensor_msgs::msg::Image output_msg;
      bridge_image.toImageMsg(output_msg);
      sensor_msgs::msg::Image::ConstSharedPtr output_ptr = std::make_shared<sensor_msgs::msg::Image>(output_msg);
     callback_(output_ptr);
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