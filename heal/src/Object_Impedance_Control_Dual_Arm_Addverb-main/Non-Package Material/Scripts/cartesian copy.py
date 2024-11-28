#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import threading
import numpy as np
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import Chain, ChainIkSolverPos_LMA, Frame, Rotation, Vector, JntArray
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Quaternion
from trac_ik_python.trac_ik import IK

# Global publisher dictionary
publishers = {}

def setup_publishers(namespaces, joint_names):
    """Pre-initialize publishers for all joints and namespaces."""
    for namespace in namespaces:
        for name in joint_names:
            topic = f'/{namespace}/{name}_position_controller/command'
            publishers[(namespace, name)] = rospy.Publisher(topic, Float64, queue_size=10)
    rospy.sleep(1)  # Allow time for publishers to set up

def load_robot_model(namespace):
    """ Load the URDF from the parameter server and construct the KDL tree """
    param_name = f'{namespace}/robot_description'
    try:
        robot_urdf = URDF.from_parameter_server(key=param_name)
        success, kdl_tree = treeFromUrdfModel(robot_urdf)
        if success:
            rospy.loginfo(f"KDL tree for {namespace} created successfully.")
            # Assuming you know the base link and tip link names
            base_link = 'base_link'  # Replace with your actual base link name
            tip_link = 'end_effector'  # Replace with your actual tip link name
            kdl_chain = kdl_tree.getChain(base_link, tip_link)
            return kdl_chain
        else:
            rospy.logerr(f"Failed to construct KDL tree for {namespace}.")
    except Exception as e:
        rospy.logerr(f"Error loading URDF from parameter server for {namespace}: {e}")
    return None

def compute_ik(x, y, z, roll, pitch, yaw, chain_base_link, chain_tip_link, urdf_param="/robot_description"):
    """ Compute inverse kinematics using TRAC-IK """
    # Initialize the IK solver
    ik_solver = IK(chain_base_link,
                   chain_tip_link,
                   urdf_string=rospy.get_param(urdf_param),
                   timeout=0.005,  # Timeout for IK solution
                   epsilon=1e-5,   # Tolerance for IK solution
                   solve_type="Speed")  # Solve type could be Speed, Distance, or Manipulation1

    # Convert RPY to Quaternion for TRAC-IK
    quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    q = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    # Create Pose for the IK solver
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = q

    # Compute IK
    solution = ik_solver.get_ik(pose.position, pose.orientation, seed_state=[0]*ik_solver.number_of_joints)  # provide a neutral seed state

    if solution:
        rospy.loginfo("IK computation successful")
        return solution
    else:
        rospy.logerr("IK computation failed.")
        return None

def move_joints_once(namespace, target_pose):
    """ Calculate joint positions using IK and publish them """
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    kdl_chain = load_robot_model(namespace)
    if kdl_chain:
        joint_positions = compute_ik(kdl_chain, *target_pose)
        if joint_positions:
            for i, name in enumerate(joint_names):
                position = Float64(joint_positions[i])
                rospy.loginfo(f"Publishing {position.data} to {namespace}: {name}")
                publishers[(namespace, name)].publish(position)

def start_thread(namespace, target_pose):
    """ Start a thread to move joints """
    thread = threading.Thread(target=move_joints_once, args=(namespace, target_pose))
    thread.start()
    return thread

box_length = 0.25
box_width = 0.2
box_height = 0.15

def main():
    rospy.init_node('joint_position_publisher_once', anonymous=True)

    # Define target poses for both robots
    #target_pose_robot1 = (0.2, 0.4, 0.25, 1.5, 2.0, 1)
    #target_pose_robot2 = (0.3, 0.5, 0.25, 1.0, 0.0, 1)

    # Robot 1's target pose to reach the left face of the box
    target_pose_robot1 = (0.15, 0.5, 0.075, 1.5, 2.0, 1)

    # Robot 2's target pose to reach the right face of the box, adjusted for its position offset
    target_pose_robot2 = (-4.87146879, 0.49999523, 0.07498561, 1.5, 2.0, 1)

    # Define namespaces and joint names
    namespaces = ['robot1', 'robot2']
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Setup all publishers
    setup_publishers(namespaces, joint_names)

    # Start sending joint positions to both robots simultaneously
    thread1 = start_thread('robot1', target_pose_robot1)
    thread2 = start_thread('robot2', target_pose_robot2)

    # Wait for both threads to complete
    thread1.join()
    thread2.join()

    rospy.spin()  # Keep the node alive if necessary

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
