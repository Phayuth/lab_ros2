import copy
from math import sin
from random import random
import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from interactive_markers import InteractiveMarkerServer
from interactive_markers import MenuHandler
import rclpy
from rosidl_runtime_py import set_message_fields
from tf2_ros.transform_broadcaster import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

node = None

server = None
menu_handler = MenuHandler()
br = None
counter = 0


def frameCallback():
    global node, counter, br
    time = node.get_clock().now()
    transform = TransformStamped()
    set_message_fields(
        transform,
        {
            "header": {"frame_id": "base_link", "stamp": time.to_msg()},
            "transform": {
                "translation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": sin(counter / 140.0) * 2.0,
                },
                "rotation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0,
                },
            },
            "child_frame_id": "moving_frame",
        },
    )
    br.sendTransform(transform)
    counter += 1


def processFeedback(feedback):
    log_prefix = f"Feedback from marker '{feedback.marker_name}' / control '{feedback.control_name}'"

    log_mouse = ""
    if feedback.mouse_point_valid:
        log_mouse = (
            f"{feedback.mouse_point.x}, {feedback.mouse_point.y}, "
            f"{feedback.mouse_point.z} in frame {feedback.header.frame_id}"
        )

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        node.get_logger().info(f"{log_prefix}: button click at {log_mouse}")
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        node.get_logger().info(
            f"{log_prefix}: menu item {feedback.menu_entry_id} clicked at {log_mouse}"
        )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        node.get_logger().info(
            f"{log_prefix}: pose changed\n"
            f"position: "
            f"{feedback.pose.position.x}, {feedback.pose.position.y}, {feedback.pose.position.z}\n"
            f"orientation: "
            f"{feedback.pose.orientation.w}, {feedback.pose.orientation.x}, "
            f"{feedback.pose.orientation.y}, {feedback.pose.orientation.z}\n"
            f"frame: {feedback.header.frame_id} "
            f"time: {feedback.header.stamp.sec} sec, "
            f"{feedback.header.stamp.nanosec} nsec"
        )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        node.get_logger().info(f"{log_prefix}: mouse down at {log_mouse}")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        node.get_logger().info(f"{log_prefix}: mouse up at {log_mouse}")


def alignMarker(feedback):
    pose = feedback.pose

    pose.position.x = round(pose.position.x - 0.5) + 0.5
    pose.position.y = round(pose.position.y - 0.5) + 0.5

    node.get_logger().info(
        f"{feedback.marker_name}: aligning position = {feedback.pose.position.x}, "
        f"{feedback.pose.position.y}, {feedback.pose.position.z} to "
        f"{pose.position.x}, {pose.position.y}, {pose.position.z}"
    )

    server.setPose(feedback.marker_name, pose)
    server.applyChanges()


def rand(min_, max_):
    return min_ + random() * (max_ - min_)


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def saveMarker(int_marker):
    server.insert(int_marker, feedback_callback=processFeedback)


def normalizeQuaternion(quaternion_msg):
    norm = (
        quaternion_msg.x**2
        + quaternion_msg.y**2
        + quaternion_msg.z**2
        + quaternion_msg.w**2
    )
    s = norm ** (-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def make6DofMarker(fixed, interaction_mode, position, show_6dof=False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {
            InteractiveMarkerControl.MOVE_3D: "MOVE_3D",
            InteractiveMarkerControl.ROTATE_3D: "ROTATE_3D",
            InteractiveMarkerControl.MOVE_ROTATE_3D: "MOVE_ROTATE_3D",
        }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof:
            int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)
    menu_handler.apply(server, int_marker.name)


def makeRandomDofMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "6dof_random_axes"
    int_marker.description = "6-DOF\n(Arbitrary Axes)"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()

    for i in range(3):
        control.orientation.w = rand(-1, 1)
        control.orientation.x = rand(-1, 1)
        control.orientation.y = rand(-1, 1)
        control.orientation.z = rand(-1, 1)
        normalizeQuaternion(control.orientation)
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

    server.insert(int_marker, feedback_callback=processFeedback)


def makeViewFacingMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "view_facing"
    int_marker.description = "View Facing 6-DOF"

    # make a control that rotates around the view axis
    control = InteractiveMarkerControl()
    control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation.w = 1.0
    control.name = "rotate"
    int_marker.controls.append(control)

    # create a box in the center which should not be view facing,
    # but move in the camera plane.
    control = InteractiveMarkerControl()
    control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.independent_marker_orientation = True
    control.name = "move"
    control.markers.append(makeBox(int_marker))
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)


def makeQuadrocopterMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "quadrocopter"
    int_marker.description = "Quadrocopter"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)


def makeChessPieceMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "chess_piece"
    int_marker.description = "Chess Piece\n(2D Move + Alignment)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    control.markers.append(makeBox(int_marker))
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, feedback_callback=processFeedback)

    # set different callback for POSE_UPDATE feedback
    server.setCallback(
        int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE
    )


def makePanTiltMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "pan_tilt"
    int_marker.description = "Pan / Tilt"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.INHERIT
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)


def makeMenuMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "context_menu"
    int_marker.description = "Context Menu\n(Right Click)"

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description = "Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    # make one control showing a box
    marker = makeBox(int_marker)
    control.markers.append(marker)
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)
    menu_handler.apply(server, int_marker.name)


def makeMovingMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "moving_frame"
    int_marker.pose.position = position
    int_marker.scale = 1.0

    int_marker.name = "moving"
    int_marker.description = "Marker Attached to a\nMoving Frame"

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append(makeBox(int_marker))
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("basic_controls")
    br = TransformBroadcaster(node)

    # create a timer to update the published transforms
    timer = node.create_timer(0.01, frameCallback)

    server = InteractiveMarkerServer(node, "basic_controls")

    menu_handler.insert("First Entry", callback=processFeedback)
    menu_handler.insert("Second Entry", callback=processFeedback)
    sub_menu_handle = menu_handler.insert("Submenu")
    menu_handler.insert(
        "First Entry", parent=sub_menu_handle, callback=processFeedback
    )
    menu_handler.insert(
        "Second Entry", parent=sub_menu_handle, callback=processFeedback
    )

    position = Point(x=-3.0, y=3.0, z=0.0)
    make6DofMarker(False, InteractiveMarkerControl.NONE, position, True)
    position = Point(x=0.0, y=3.0, z=0.0)
    make6DofMarker(True, InteractiveMarkerControl.NONE, position, True)
    position = Point(x=3.0, y=3.0, z=0.0)
    makeRandomDofMarker(position)
    position = Point(x=-3.0, y=0.0, z=0.0)
    make6DofMarker(False, InteractiveMarkerControl.ROTATE_3D, position, False)
    position = Point(x=0.0, y=0.0, z=0.0)
    make6DofMarker(False, InteractiveMarkerControl.MOVE_ROTATE_3D, position, True)
    position = Point(x=3.0, y=0.0, z=0.0)
    make6DofMarker(False, InteractiveMarkerControl.MOVE_3D, position, False)
    position = Point(x=-3.0, y=-3.0, z=0.0)
    makeViewFacingMarker(position)
    position = Point(x=0.0, y=-3.0, z=0.0)
    makeQuadrocopterMarker(position)
    position = Point(x=3.0, y=-3.0, z=0.0)
    makeChessPieceMarker(position)
    position = Point(x=-3.0, y=-6.0, z=0.0)
    makePanTiltMarker(position)
    position = Point(x=0.0, y=-6.0, z=0.0)
    makeMovingMarker(position)
    position = Point(x=3.0, y=-6.0, z=0.0)
    makeMenuMarker(position)

    server.applyChanges()

    rclpy.spin(node)
    server.shutdown()
