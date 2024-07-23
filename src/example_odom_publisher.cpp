// ROS2
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
// STL
#include <string>

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
    : rclcpp::Node("example_odom_publisher_node")
    , pub_{nullptr}
    , timer_{nullptr}
    , count_{0}
  {
    std::chrono::milliseconds odom_period{500};
    std::chrono::milliseconds lease_delta{20};
    // Init publisher
    if (!pub_)
    {
      // The granted lease is essentially infite here, i.e., only reader/watchdog will notify
      // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life > heartbeat.
      rclcpp::QoS qos_profile{1};
      qos_profile
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
        .liveliness_lease_duration(odom_period + lease_delta)
        .deadline(odom_period + lease_delta);

      pub_ = create_publisher<std_msgs::msg::Int32>("odom", qos_profile);
    }

    // init timer
    if (!timer_)
    {
      timer_ = create_wall_timer(odom_period,
          [this]() -> void
          {
            if (20 < count_ && !timer_->is_canceled()) return timer_->cancel();
            count_++;
            std_msgs::msg::Int32 msg;
            msg.data = count_;
            RCLCPP_INFO_STREAM(get_logger(), "odom published: " << msg.data);
            pub_->publish(msg);
          }
        );
    }
  }
  virtual ~OdomPublisher() {}

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t count_;
};

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
