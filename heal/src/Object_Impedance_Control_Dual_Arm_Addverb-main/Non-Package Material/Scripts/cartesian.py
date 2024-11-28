#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import threading
import numpy as np
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import Chain, ChainIkSolverPos_LMA, Frame, Rotation, Vector, JntArray

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
            base_link = 'base_link'  
            tip_link = 'end_effector'  
            kdl_chain = kdl_tree.getChain(base_link, tip_link)
            return kdl_chain
        else:
            rospy.logerr(f"Failed to construct KDL tree for {namespace}.")
    except Exception as e:
        rospy.logerr(f"Error loading URDF from parameter server for {namespace}: {e}")
    return None

def compute_ik(kdl_chain, x, y, z, roll, pitch, yaw):
    """ Compute inverse kinematics using KDL """
    ik_solver = ChainIkSolverPos_LMA(kdl_chain)
    target_frame = Frame(Rotation.RPY(roll, pitch, yaw), Vector(x, y, z))
    joint_positions = JntArray(kdl_chain.getNrOfJoints())
    if ik_solver.CartToJnt(joint_positions, target_frame, joint_positions) >= 0:
        return joint_positions
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

def main():
    rospy.init_node('joint_position_publisher_once', anonymous=True)

    # Define target poses for both robots
    #target_pose_robot1 = (0.2, 0.4, 0.25, 1.5, 2.0, 1)
    #target_pose_robot2 = (0.3, 0.5, 0.25, 1.0, 0.0, 1)

    target_pose_robot1 = (0.040, 0.45, 0.12, 1.5, 2.0, 1.5)

   
    target_pose_robot2 = (0.080, 0.45, 0.12, 1.5, 2.0, -1.6)



    namespaces = ['robot1', 'robot2']
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


    setup_publishers(namespaces, joint_names)

    thread1 = start_thread('robot1', target_pose_robot1)
    #rospy.sleep(2)
    thread2 = start_thread('robot2', target_pose_robot2)

    thread1.join()
    thread2.join()

    #rospy.spin()  # Keep the node alive if necessary

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
