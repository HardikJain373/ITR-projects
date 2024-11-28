#!/usr/bin/env python
# This version fof code the motion still does not occur a

import rospy
from std_msgs.msg import Float64
import threading

def move_joints_once(namespace, joint_positions):
    rospy.init_node('joint_position_publisher_once', anonymous=True, disable_signals=True)
    
    publishers = []
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    for name in joint_names:
        # Format the topic with the namespace and joint name
        topic = '/{}/{}_position_controller/command'.format(namespace, name)
        pub = rospy.Publisher(topic, Float64, queue_size=10)
        publishers.append(pub)

    rospy.sleep(1)  # Ensure publishers are setup

    # Publish predefined positions for each joint
    for i, pub in enumerate(publishers):
        position = Float64()
        position.data = joint_positions[i]
        rospy.loginfo("Publishing {} to {}: {}".format(position.data, namespace, joint_names[i]))
        pub.publish(position)
    
    rospy.sleep(2)  # Time for the last command to be sent
    rospy.loginfo("All joint positions for {} have been set.".format(namespace))

def start_thread(namespace, joint_positions):
    thread = threading.Thread(target=move_joints_once, args=(namespace, joint_positions))
    thread.start()
    return thread

def main():
    # Define joint positions for robot1
    joint_positions_robot1 = [-1.57, 0.0, 1.57, 3.14, -1.57, 0.0]
    # Define joint positions for robot2
    joint_positions_robot2 = [1.57, 0.0, -1.57, -3.14, 1.57, 0.0]

    # Start sending joint positions to robot1 and robot2 simultaneously
    thread1 = start_thread('robot1', joint_positions_robot1)
    thread2 = start_thread('robot2', joint_positions_robot2)

    # Wait for both threads to complete
    thread1.join()
    thread2.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
