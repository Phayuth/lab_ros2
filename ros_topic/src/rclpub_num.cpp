#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

class PublishInterger : public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
        size_t int_data{0};

        void timer_callback() {
            std_msgs::msg::Int64 i;
            int_data = this->get_parameter("set_int").as_int();
            i.data = int_data;

            pub_->publish(i);
            RCLCPP_INFO(this->get_logger(), "Int data [%i] is published", i);
        };

    public:
        PublishInterger() : Node("int_pub") {
            pub_ = this->create_publisher<std_msgs::msg::Int64>("int", 10);
            using namespace std::chrono_literals;
            timer_ = this->create_wall_timer(500ms, std::bind(&PublishInterger::timer_callback, this));

            this->declare_parameter("set_int", 0);
        }
};

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublishInterger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
