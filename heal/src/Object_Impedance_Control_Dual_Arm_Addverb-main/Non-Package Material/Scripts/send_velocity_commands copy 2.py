#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from kdl_parser_py.urdf import treeFromParam
import PyKDL as kdl

def linear_trajectory(t, start_pose, end_pose, duration):
    # Linear interpolation between start and end poses
    if t > duration:
        t = duration
    return start_pose + (end_pose - start_pose) * (t / duration)

def compute_joint_velocities(jacobian, end_effector_velocity):
    # Use the pseudo-inverse of the Jacobian to compute joint velocities
    return np.linalg.pinv(jacobian) @ end_effector_velocity

def send_velocity_commands():
    rospy.init_node('cartesian_velocity_command_publisher')

    # Load robot URDF and create KDL chain
    (ok, tree) = treeFromParam("/robot_description")
    if not ok:
        rospy.logerr("Failed to parse URDF")
        return
    
    chain = tree.getChain("base_link", "end_effector")

    # Define the joint publishers
    joint_publishers = [
        rospy.Publisher('/joint1_velocity_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint2_velocity_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint3_velocity_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint4_velocity_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint5_velocity_controller/command', Float64, queue_size=10),
        rospy.Publisher('/joint6_velocity_controller/command', Float64, queue_size=10)
    ]

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()

    # Define start and end poses
    start_pose = np.array([0.110, 0.45, 0.4, 1.5, 2.0, 1.5])
    end_pose = np.array([0.110, 0.45, 0.6, 1.5, 2.0, 1.5])
    duration = 5.0  # Duration of the trajectory in seconds

    # Create KDL solvers
    fk_solver = kdl.ChainFkSolverPos_recursive(chain)
    jac_solver = kdl.ChainJntToJacSolver(chain)
    joint_angles = kdl.JntArray(chain.getNrOfJoints())
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        # Compute desired end-effector position
        desired_position = linear_trajectory(elapsed_time, start_pose[:3], end_pose[:3], duration)

        # Compute desired end-effector velocity
        if elapsed_time < duration:
            desired_velocity = (end_pose[:3] - start_pose[:3]) / duration
        else:
            desired_velocity = np.zeros(3)

        # Perform FK to get current end-effector position and Jacobian
        end_effector_frame = kdl.Frame()
        fk_solver.JntToCart(joint_angles, end_effector_frame)
        jacobian = kdl.Jacobian(chain.getNrOfJoints())
        jac_solver.JntToJac(joint_angles, jacobian)

        # Convert Jacobian to numpy array
        jacobian_array = np.zeros((6, chain.getNrOfJoints()))
        for i in range(6):
            for j in range(chain.getNrOfJoints()):
                jacobian_array[i, j] = jacobian[i, j]

        # Compute joint velocities
        joint_velocities = compute_joint_velocities(jacobian_array[:3, :], desired_velocity)  # Use only the linear part

        # Publish the joint velocities
        for i in range(chain.getNrOfJoints()):
            joint_publishers[i].publish(joint_velocities[i])

        rate.sleep()

if __name__ == '__main__':
    try:
        send_velocity_commands()
    except rospy.ROSInterruptException:
        pass
