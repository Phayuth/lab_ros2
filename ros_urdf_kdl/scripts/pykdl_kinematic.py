"""
- [1] http://docs.ros.org/en/diamondback/api/kdl/html/python/
- [2] http://docs.ros.org/en/indigo/api/orocos_kdl/html/
"""

import PyKDL
import numpy as np
import urdfkdl


def make_kdl_from_dh():
    chain = PyKDL.Chain()

    a = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
    alpha = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]
    d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
    theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    for i in range(len(theta)):
        joint = PyKDL.Joint(PyKDL.Joint.RotZ)
        frame = PyKDL.Frame(PyKDL.Rotation.RotZ(theta[i]), PyKDL.Vector(a[i], -d[i], -alpha[i]))
        segment = PyKDL.Segment(joint, frame)
        chain.addSegment(segment)

    return chain


def make_kdl_from_urdf():
    done, tree = urdfkdl.treeFromFile("./ur5e.urdf")

    root = "ur5e_base_link"
    tip = "ur5e_tool0"
    chain = tree.getChain(root, tip)

    return chain


def make_kdl_from_urdf2():
    done, tree = urdfkdl.treeFromFile("./ur5e_onsga.urdf")

    root = "ur5e_base_link"
    tip = "onrobotsg_tip"
    chain = tree.getChain(root, tip)

    return chain


# chain = make_kdl_from_dh()
# chain = make_kdl_from_urdf()
chain = make_kdl_from_urdf2()

fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)

print(f"Number of Joints: {chain.getNrOfJoints()}")

joint_positions = PyKDL.JntArray(chain.getNrOfJoints())
joint_positions[0] = 0.0
joint_positions[1] = 0.0
joint_positions[2] = 0.0
joint_positions[3] = 0.0
joint_positions[4] = 0.0
joint_positions[5] = 0.0

end_effector_frame = PyKDL.Frame()
fk_solver.JntToCart(joint_positions, end_effector_frame)

print(f"End-Effector Position: {end_effector_frame.p}")
print(f"End-Effector Orientation: {end_effector_frame.M}")


# # Perform forward kinematics
# end_effector_frame = PyKDL.Frame()
# fk_solver.JntToCart(joint_positions, end_effector_frame)

# # Print the end-effector position and orientation
# print(f"End-Effector Position: {end_effector_frame.p}")
# print(f"End-Effector Orientation: {end_effector_frame.M.GetRPY()}")

# # Create the inverse kinematics solver
# ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)

# # Set target frame
# target_frame = PyKDL.Frame()
# target_frame.p = PyKDL.Vector(
#     end_effector_frame.p[0],
#     end_effector_frame.p[1],
#     end_effector_frame.p[2],
# )
# target_frame.M = PyKDL.Rotation.RPY(
#     end_effector_frame.M.GetRPY()[0],
#     end_effector_frame.M.GetRPY()[1],
#     end_effector_frame.M.GetRPY()[2],
# )

# # Perform inverse kinematics
# initial_joint_positions = PyKDL.JntArray(chain.getNrOfJoints())
# initial_joint_positions[0] = 0.0  # Joint 1 initial position in radians
# initial_joint_positions[1] = 0.0  # Joint 2 initial position in radians
# initial_joint_positions[2] = 0.0  # Joint 3 initial position in radians
# initial_joint_positions[3] = 0.0  # Joint 4 initial position in radians
# initial_joint_positions[4] = 0.0  # Joint 5 initial position in radians
# initial_joint_positions[5] = 0.0  # Joint 6 initial position in radians

# final_joint_positions = PyKDL.JntArray(chain.getNrOfJoints())
# ik_solver.CartToJnt(initial_joint_positions, target_frame, final_joint_positions)

# # Print the final joint positions
# print("Final Joint Positions:")
# for i in range(chain.getNrOfJoints()):
#     print("Joint {}: {:.4f} rad".format(i + 1, final_joint_positions[i]))

# # Set joint positions
# joint_positions = PyKDL.JntArray(chain.getNrOfJoints())
# joint_positions[0] = final_joint_positions[0]  # Joint 1 position in radians
# joint_positions[1] = final_joint_positions[1]  # Joint 2 position in radians
# joint_positions[2] = final_joint_positions[2]  # Joint 3 position in radians
# joint_positions[3] = final_joint_positions[3]  # Joint 4 position in radians
# joint_positions[4] = final_joint_positions[4]  # Joint 5 position in radians
# joint_positions[5] = final_joint_positions[5]  # Joint 6 position in radians

# # Perform forward kinematics
# end_effector_frame = PyKDL.Frame()
# fk_solver.JntToCart(joint_positions, end_effector_frame)

# # Print the end-effector position and orientation
# print(f"End-Effector Position: {end_effector_frame.p}")
# print(f"End-Effector Orientation: {end_effector_frame.M.GetRPY()}")


# print(dir(PyKDL))
# print(help(PyKDL))
