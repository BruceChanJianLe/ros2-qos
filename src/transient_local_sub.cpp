#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
// STL
#include <chrono>

class TransientLocalSub : public rclcpp::Node
{
public:
    TransientLocalSub()
    : rclcpp::Node("tl_sub_node")
    , count(0)
    {
        tl_sub_ = this->create_subscription<std_msgs::msg::Int64>("tl_topic", rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Int64::SharedPtr msg)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Receive count " << msg->data);
            }
        );
    }

    ~TransientLocalSub() {}
private:
    int64_t count;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr tl_sub_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TransientLocalSub>();

    rclcpp::spin(node);

    return 0;
}
