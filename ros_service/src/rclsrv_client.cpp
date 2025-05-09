#include <example_interfaces/srv/add_two_ints.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

class MinimalClient : public rclcpp::Node {
    private:
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
        std::thread thread_;

    public:
        MinimalClient() : Node("client") {
            thread_ = std::thread(
                std::bind(&MinimalClient::call_add_two_int, this, 4, 1));
        }

        void call_add_two_int(int a, int b) {
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
                "add_srv");

            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(),
                            "The server is currently unavailable! waiting...");
            };

            auto req =
                std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            req->a = a;
            req->b = b;

            auto future = client_->async_send_request(req);

            try {
                auto res = future.get();
                RCLCPP_INFO(this->get_logger(), "The answer %i", res->sum);
            } catch (const std::exception &e) {
                std::cerr << e.what() << '\n';
                RCLCPP_INFO(this->get_logger(), "Service call failed!");
            }
        }
};

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
