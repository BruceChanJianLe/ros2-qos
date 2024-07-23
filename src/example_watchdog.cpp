// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
// STL
#include <chrono>

namespace util
{
  class ExampleWatchDog : public rclcpp::Node
  {
  public:
    explicit ExampleWatchDog()
      : rclcpp::Node("example_watchdog_node")
      , topic_name_{"odom"}
      , qos_profile_{10}
      , sub_{nullptr}
      , pub_{nullptr}
      , timer_{nullptr}
      , is_odom_updated_{false}
    {
      lease_duration_ = std::chrono::milliseconds{550};

      // Configure watchdog
      qos_profile_
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
        .liveliness_lease_duration(lease_duration_);
      sub_options_.event_callbacks.liveliness_callback =
        [this](rclcpp::QOSLivelinessChangedInfo &event) -> void
        {
          // This is the watchdog callback
          if (event.alive_count == 0)
          {
            is_odom_updated_ = false;
            RCLCPP_WARN_STREAM(get_logger(), "Odom topic liveliness changed event triggered");
            RCLCPP_WARN_STREAM(get_logger(), "Stopping robot!");
          }
          else
          {
            RCLCPP_INFO_STREAM(get_logger(), "Odom topic is alive!");
          }
        };

      // Creaet safety publisher
      if (!pub_)
      {
        pub_ = create_publisher<std_msgs::msg::Int32>("safety_output", 1);
      }
      // Create odom subscriber
      if (!sub_)
      {
        sub_ = create_subscription<std_msgs::msg::Int32>(
            topic_name_,
            qos_profile_,
            [this](const std_msgs::msg::Int32::SharedPtr msg) -> void
            {
              RCLCPP_INFO_STREAM(get_logger(), "Odom msg " << msg->data);
              is_odom_updated_ = true;
            },
            sub_options_
          );
      }

      // Create time to publish safety value
      if (!timer_)
      {
        timer_ = create_wall_timer(std::chrono::seconds(1),
            [this]()
            {
              std_msgs::msg::Int32 msg;
              if (is_odom_updated_)
              {
                msg.data = 0;
              }
              else
              {
                msg.data = 3;
              }
              RCLCPP_INFO_STREAM(get_logger(), "Safety Msg: " << msg.data);
              pub_->publish(msg);
            }
          );
      }
    }
    virtual ~ExampleWatchDog() {}

  private:
    std::chrono::milliseconds lease_duration_;
    const std::string topic_name_;
    rclcpp::QoS qos_profile_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::SubscriptionOptions sub_options_;

    bool is_odom_updated_;
  };

} // util

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<util::ExampleWatchDog>());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
