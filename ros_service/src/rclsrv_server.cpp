#include <example_interfaces/srv/add_two_ints.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

class MinimalServer : public rclcpp::Node {
    private:
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

        void callback(
            const example_interfaces::srv::AddTwoInts::Request::SharedPtr req,
            const example_interfaces::srv::AddTwoInts::Response::SharedPtr res) {
            res->sum = req->a + req->b;
            RCLCPP_INFO(
                this->get_logger(),
                "I heard a request to add [%i] and [%i] and I returned [%i].",
                req->a,
                req->b,
                res->sum);
        }

    public:
        MinimalServer() : Node("server") {
            server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "add_srv",
                std::bind(&MinimalServer::callback,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));
        }
};

int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}