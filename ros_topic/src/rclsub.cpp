#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalSubscriber : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

        void topic_callback(const std_msgs::msg::String::SharedPtr std_msgs) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std_msgs->data.c_str());
        }

    public:
        MinimalSubscriber() : Node("minimal_subscriber") { // the callback function have 1 arg. so we use placeholder _1
            subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}