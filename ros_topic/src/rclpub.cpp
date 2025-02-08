#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalPublisher : public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_ = 0;

        void timer_callback() {
            std_msgs::msg::String message;
            message.data = "Hello, world! " + std::to_string(count_++);
            publisher_->publish(message);

            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        }

    public:
        MinimalPublisher() : Node("minimal_publisher") { // constructor
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

            using namespace std::chrono_literals;
            timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}