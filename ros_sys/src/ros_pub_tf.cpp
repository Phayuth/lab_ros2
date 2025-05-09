#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class TransformPublisher : public rclcpp::Node {
    public:
        TransformPublisher() : Node("transform_publisher") {
            static_broadcaster_ =
                std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            dynamic_broadcaster_ =
                std::make_shared<tf2_ros::TransformBroadcaster>(this);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&TransformPublisher::publish_dynamic_transform, this));
        }

    private:
        void publish_static_transform() {
            geometry_msgs::msg::TransformStamped static_transform;
            static_transform.header.stamp = this->get_clock()->now();
            static_transform.header.frame_id = "world";
            static_transform.child_frame_id = "static_frame";
            static_transform.transform.translation.x = 1.0;
            static_transform.transform.translation.y = 0.0;
            static_transform.transform.translation.z = 0.0;
            static_transform.transform.rotation.x = 0.0;
            static_transform.transform.rotation.y = 0.0;
            static_transform.transform.rotation.z = 0.0;
            static_transform.transform.rotation.w = 1.0;

            static_broadcaster_->sendTransform(static_transform);
        }

        void publish_dynamic_transform() {
            geometry_msgs::msg::TransformStamped dynamic_transform;
            dynamic_transform.header.stamp = this->get_clock()->now();
            dynamic_transform.header.frame_id = "world";
            dynamic_transform.child_frame_id = "dynamic_frame";
            dynamic_transform.transform.translation.x =
                sin(this->now().nanoseconds() / 1e9);
            dynamic_transform.transform.translation.y =
                cos(this->now().nanoseconds() / 1e9);
            dynamic_transform.transform.translation.z = 0.0;
            dynamic_transform.transform.rotation.x = 0.0;
            dynamic_transform.transform.rotation.y = 0.0;
            dynamic_transform.transform.rotation.z = 0.0;
            dynamic_transform.transform.rotation.w = 1.0;

            dynamic_broadcaster_->sendTransform(dynamic_transform);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisher>());
    rclcpp::shutdown();
    return 0;
}