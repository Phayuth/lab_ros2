#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster_node.hpp>
#include <tf2_ros/transform_listener.h>

// tf lookup transform
class ROS2TF2Lookup : public rclcpp::Node {
    private:
    public:
        ROS2TF2Lookup() : Node("tf2_lookup") {
            // create a tf2 buffer
            tf2_ros::Buffer buffer(this->get_clock());
            // create a tf2 listener
            tf2_ros::TransformListener listener(buffer, this, false);
            // wait for the first transform
            buffer.canTransform("odom", "base_link", rclcpp::Time(0));

            // lookup the transform
            geometry_msgs::msg::TransformStamped transform_msg;
            try {
                transform_msg = buffer.lookupTransform("odom", "base_link", rclcpp::Time(0));
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), ex.what());
            }
        }
};