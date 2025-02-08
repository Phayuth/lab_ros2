#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster_node.hpp>

class ROS2Transform : rclcpp::Node {
    private:
    public:
        ROS2Transform() : Node("tf_node") {

            geometry_msgs::msg::TransformStamped odom2laser_msg;
            odom2laser_msg.header.frame_id = "parent";
            odom2laser_msg.child_frame_id = "child";
            odom2laser_msg.transform.rotation.w = 1.0;
            odom2laser_msg.transform.rotation.x = 0.0;
            odom2laser_msg.transform.rotation.y = 0.0;
            odom2laser_msg.transform.rotation.z = 0.0;
            odom2laser_msg.transform.translation.x = 1.0;
            odom2laser_msg.transform.translation.y = 1.0;
            odom2laser_msg.transform.translation.z = 1.0;

            tf2::Stamped<tf2::Transform> odom2laser;
            tf2::fromMsg(odom2laser_msg, odom2laser);

            tf2::Transform laser2object;
            laser2object.setOrigin(tf2::Vector3(1.3, 0.0, 0.0));
            laser2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

            tf2::Transform odom2object = odom2laser * laser2object;

            geometry_msgs::msg::TransformStamped odom2object_msg;
            odom2object_msg.transform = tf2::toMsg(odom2object);
            odom2object_msg.header.stamp = now();
            odom2object_msg.header.frame_id = "odom";
            odom2object_msg.child_frame_id = "detected_obstacle";
        }
};

int main(int argc, char const *argv[]) {
    return 0;
}
