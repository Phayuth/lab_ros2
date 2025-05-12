from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
import tf2_geometry_msgs
import tf2_ros
import tf2_py
import tf2_kdl

HcamTObaseTFS = TransformStamped()
HcamTObaseTFS.header.frame_id = "base_link"
HcamTObaseTFS.child_frame_id = "camera_link"
HcamTObaseTFS.transform.translation.x = 0.5
HcamTObaseTFS.transform.translation.y = 0.5
HcamTObaseTFS.transform.translation.z = 0.5
HcamTObaseTFS.transform.rotation.x = 0.0
HcamTObaseTFS.transform.rotation.y = 0.0
HcamTObaseTFS.transform.rotation.z = 0.0
HcamTObaseTFS.transform.rotation.w = 1.0
print(f"> HcamTObaseTFS: {HcamTObaseTFS}")

HgraspTOcamPose = Pose()
HgraspTOcamPose.position.x = 0.5
HgraspTOcamPose.position.y = 0.5
HgraspTOcamPose.position.z = 0.0
HgraspTOcamPose.orientation.x = 0.0
HgraspTOcamPose.orientation.y = 0.0
HgraspTOcamPose.orientation.z = 0.0
HgraspTOcamPose.orientation.w = 1.0
print(f"> HgraspTOcamPose: {HgraspTOcamPose}")

print("--------------------------------------------")
HgraspTObase = tf2_geometry_msgs.do_transform_pose(
    pose=HgraspTOcamPose, transform=HcamTObaseTFS
)
print(f"> HgraspTObase: {HgraspTObase}")


# Hgrasptobase = Hcamtobase @ Hgrasptocam
# Hpregrasptobase = Hcamtobase @ Hpregrasptocam
