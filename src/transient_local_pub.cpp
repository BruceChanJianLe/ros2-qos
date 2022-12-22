#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
// STL
#include <chrono>

class TransientLocalPub : public rclcpp::Node
{
public:
    TransientLocalPub()
    : rclcpp::Node("tl_pub_node")
    , count(10)
    {
        tl_pub_ = this->create_publisher<std_msgs::msg::Int64>("tl_topic", rclcpp::QoS(1).reliable().transient_local());
        std_msgs::msg::Int64 msg;
        msg.set__data(count);
        this->tl_pub_->publish(msg);
    }

    ~TransientLocalPub() {}
private:
    int64_t count;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr tl_pub_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TransientLocalPub>();

    rclcpp::spin(node);

    return 0;
}
