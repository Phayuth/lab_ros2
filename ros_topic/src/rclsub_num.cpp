#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

class SubscribeInterger : public rclcpp::Node {
    private:
        rclcpp::SubscriptionBase::SharedPtr sub_;

        void subscribe_callback(const std_msgs::msg::Int64::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "I heard [%i].", msg->data);
        }

    public:
        SubscribeInterger() : Node("int_sub") {
            sub_ = this->create_subscription<std_msgs::msg::Int64>(
                "int",
                10,
                std::bind(&SubscribeInterger::subscribe_callback,
                          this,
                          std::placeholders::_1));
        }
};

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscribeInterger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
