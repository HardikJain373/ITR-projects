#!/usr/bin/env python

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def send_trajectory_to_robot2():
    rospy.init_node('send_trajectory_to_robot2', anonymous=True)

    # Create a trajectory action client for robot2
    client = actionlib.SimpleActionClient('/robot2/heal2_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for action server to start for robot2.")
    # Wait for the server with a timeout of 5 seconds
    if client.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Action server is up for robot2!")
    else:
        rospy.logerr("Action server not available for robot2.")
        return

    # Define the joint names for robot2 according to your URDF and controller configuration
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Create a new trajectory for robot2
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    # Define two points in this trajectory for robot2
    point1 = JointTrajectoryPoint()
    point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point1.time_from_start = rospy.Duration(1)  # 1 second to reach this point

    point2 = JointTrajectoryPoint()
    point2.positions = [1.57, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example target positions for each joint
    point2.time_from_start = rospy.Duration(3)  # 2 more seconds to reach this point, 3 seconds total

    # Add points to the trajectory
    trajectory.points.append(point1)
    trajectory.points.append(point2)

    # Send the trajectory
    rospy.loginfo("Sending trajectory to robot2")
    goal = FollowJointTrajectoryGoal(trajectory=trajectory)
    client.send_goal(goal)
    client.wait_for_result()

    # Check the result
    action_state = client.get_state()
    if action_state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Trajectory execution for robot2 finished successfully.")
    else:
        rospy.logwarn("Trajectory execution for robot2 failed with state: %d" % action_state)

if __name__ == '__main__':
    try:
        send_trajectory_to_robot2()
    except rospy.ROSInterruptException:
        rospy.logerr("Node terminated.")
