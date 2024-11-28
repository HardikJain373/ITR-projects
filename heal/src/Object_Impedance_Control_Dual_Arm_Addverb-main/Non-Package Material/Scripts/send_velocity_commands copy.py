#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np

def generate_trajectory(t):
    # Define your trajectory function here
    # For simplicity, we'll use a sinusoidal trajectory
    amplitude = 1.0
    frequency = 0.5
    return amplitude * np.sin(2 * np.pi * frequency * t)

def send_velocity_commands():
    rospy.init_node('velocity_command_publisher')

    # Publishers for each joint
    pub_joint1 = rospy.Publisher('/joint1_velocity_controller/command', Float64, queue_size=10)
    pub_joint2 = rospy.Publisher('/joint2_velocity_controller/command', Float64, queue_size=10)
    pub_joint3 = rospy.Publisher('/joint3_velocity_controller/command', Float64, queue_size=10)
    pub_joint4 = rospy.Publisher('/joint4_velocity_controller/command', Float64, queue_size=10)
    pub_joint5 = rospy.Publisher('/joint5_velocity_controller/command', Float64, queue_size=10)
    pub_joint6 = rospy.Publisher('/joint6_velocity_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        # Compute the desired velocity based on the trajectory
        velocity = generate_trajectory(elapsed_time)

        # Publishing the computed velocity to all joints
        rospy.loginfo(f"Sending velocity {velocity} to all joints at time {elapsed_time}")
        pub_joint1.publish(velocity)
        pub_joint2.publish(velocity)
        pub_joint3.publish(velocity)
        pub_joint4.publish(velocity)
        pub_joint5.publish(velocity)
        pub_joint6.publish(velocity)

        rate.sleep()

if __name__ == '__main__':
    try:
        send_velocity_commands()
    except rospy.ROSInterruptException:
        pass
