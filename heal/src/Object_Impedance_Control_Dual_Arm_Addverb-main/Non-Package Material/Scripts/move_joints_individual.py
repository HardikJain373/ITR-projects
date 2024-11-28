#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def move_joints_once():
    rospy.init_node('joint_position_publisher_once', anonymous=True)
    
    publishers = []
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    for name in joint_names:
        pub = rospy.Publisher('/{}_position_controller/command'.format(name), Float64, queue_size=10)
        publishers.append(pub)

    # Predefined positions for each joint (in radians or meters)
    joint_positions = [
        0.0,   # Position for joint1
        0.0,   # Position for joint2
        1.57,  # Position for joint3
        0.0,   # Position for joint4
        0.6,   # Position for joint5
        3.14    # Position for joint6
    ]

    rospy.sleep(1)

    for i, pub in enumerate(publishers):
        position = Float64()
        position.data = joint_positions[i]
        rospy.loginfo("Publishing position {} for {}".format(position.data, joint_names[i]))
        pub.publish(position)
    
    rospy.sleep(1)
    rospy.loginfo("All joint positions have been set. Shutting down.")
    rospy.signal_shutdown("Finished setting initial joint positions.")

if __name__ == '__main__':
    try:
        move_joints_once()
    except rospy.ROSInterruptException:
        pass
