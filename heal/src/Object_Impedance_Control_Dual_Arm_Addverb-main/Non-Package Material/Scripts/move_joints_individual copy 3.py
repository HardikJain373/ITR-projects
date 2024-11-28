#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import threading

# Global publisher dictionary
publishers = {}

def setup_publishers(namespaces, joint_names):
    """Pre-initialize publishers for all joints and namespaces."""
    for namespace in namespaces:
        for name in joint_names:
            topic = '/{}/{}_position_controller/command'.format(namespace, name)
            publishers[(namespace, name)] = rospy.Publisher(topic, Float64, queue_size=10)
    # Allow some time for all publishers to be set up
    rospy.sleep(1)

def move_joints_once(namespace, joint_positions):
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    # Publish predefined positions for each joint
    for i, name in enumerate(joint_names):
        position = Float64(joint_positions[i])
        rospy.loginfo("Publishing {} to {}: {}".format(position.data, namespace, name))
        publishers[(namespace, name)].publish(position)

def start_thread(namespace, joint_positions):
    """Function to start a thread for joint position commands."""
    thread = threading.Thread(target=move_joints_once, args=(namespace, joint_positions))
    thread.start()
    return thread



def main():
    rospy.init_node('joint_position_publisher_once', anonymous=True)

    # Define joint positions for both robots
    joint_positions_robot1 = [-1.57, 0.0, 1.57, 3.14, -1.57, 0.0]
    joint_positions_robot2 = [1.57, 0.0, -1.57, -3.14, 1.57, 0.0]

    # joint_positions_robot1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # joint_positions_robot2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Define namespaces and joint names
    namespaces = ['robot1', 'robot2']
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Setup all publishers
    setup_publishers(namespaces, joint_names)

    # Start sending joint positions to both robots simultaneously
    thread1 = start_thread('robot1', joint_positions_robot1)
    thread2 = start_thread('robot2', joint_positions_robot2)

    # Wait for both threads to complete
    thread1.join()
    thread2.join()

    rospy.spin()  # Keep the node alive if necessary

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
