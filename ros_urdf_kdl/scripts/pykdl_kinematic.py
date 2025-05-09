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
        frame = PyKDL.Frame(
            PyKDL.Rotation.RotZ(theta[i]), PyKDL.Vector(a[i], -d[i], -alpha[i])
        )
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
chain = make_kdl_from_urdf()
# chain = make_kdl_from_urdf2()
numjoints = chain.getNrOfJoints()

fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)


# Forward Kinematics
joint_positions = PyKDL.JntArray(numjoints)
joint_positions[0] = 0.0
joint_positions[1] = -1.57
joint_positions[2] = 1.57
joint_positions[3] = 0.0
joint_positions[4] = 1.57
joint_positions[5] = 1.57

tool0INbaselink = PyKDL.Frame()
fk_solver.JntToCart(joint_positions, tool0INbaselink)

print(f"End-Effector Position: {tool0INbaselink.p}")
print(f"End-Effector Orientation: {tool0INbaselink.M}")

# Perform inverse kinematics
target_frame = PyKDL.Frame()
target_frame.p = PyKDL.Vector(
    tool0INbaselink.p[0],
    tool0INbaselink.p[1],
    tool0INbaselink.p[2],
)
target_frame.M = PyKDL.Rotation.RPY(
    tool0INbaselink.M.GetRPY()[0],
    tool0INbaselink.M.GetRPY()[1],
    tool0INbaselink.M.GetRPY()[2],
)

initial_joint_positions = PyKDL.JntArray(numjoints)
final_joint_positions = PyKDL.JntArray(numjoints)
initial_joint_positions[0] = 0.0  # Joint 1 initial position in radians
initial_joint_positions[1] = 0.0  # Joint 2 initial position in radians
initial_joint_positions[2] = 0.0  # Joint 3 initial position in radians
initial_joint_positions[3] = 0.0  # Joint 4 initial position in radians
initial_joint_positions[4] = 0.0  # Joint 5 initial position in radians
initial_joint_positions[5] = 0.0  # Joint 6 initial position in radians
ik_solver.CartToJnt(initial_joint_positions, target_frame, final_joint_positions)

print("Final Joint Positions:")
for i in range(numjoints):
    print("Joint {}: {:.4f} rad".format(i + 1, final_joint_positions[i]))


# Wrench transformation
tool0force = PyKDL.Vector(0.0, 0.0, -50.0)
tool0torque = PyKDL.Vector(0.0, 0.0, 0.0)
tool0wrench = PyKDL.Wrench(tool0force, tool0torque)

baselinkwrench = PyKDL.Wrench()
baselinkwrench = tool0INbaselink * tool0wrench
print("Wrench in base_link frame:")
print(f"Force: {baselinkwrench.force}")
print(f"Torque: {baselinkwrench.torque}")
