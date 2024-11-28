#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def send_velocity_commands():
    rospy.init_node('velocity_command_publisher')

    # Publishers for each joint
    pub_joint1 = rospy.Publisher('/joint1_velocity_controller/command', Float64, queue_size=10)
    pub_joint2 = rospy.Publisher('/joint2_velocity_controller/command', Float64, queue_size=10)
    pub_joint3 = rospy.Publisher('/joint3_velocity_controller/command', Float64, queue_size=10)
    pub_joint4 = rospy.Publisher('/joint4_velocity_controller/command', Float64, queue_size=10)
    pub_joint5 = rospy.Publisher('/joint5_velocity_controller/command', Float64, queue_size=10)
    pub_joint6 = rospy.Publisher('/joint6_velocity_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(5)  # 10 Hz

    def shutdown_hook():
        # Function to run on node shutdown
        zero_velocity = 0.0000000000000
        rospy.loginfo("Shutting down: Sending zero velocity to all joints")
        pub_joint1.publish(zero_velocity)
        pub_joint2.publish(zero_velocity)
        pub_joint3.publish(zero_velocity)
        pub_joint4.publish(zero_velocity)
        pub_joint5.publish(zero_velocity)
        pub_joint6.publish(zero_velocity)

    # Register the shutdown hook
    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        # Test velocity command
        velocity = -0.1  # Set the desired velocity here
        rospy.loginfo(f"Sending velocity {velocity} to all joints")
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
