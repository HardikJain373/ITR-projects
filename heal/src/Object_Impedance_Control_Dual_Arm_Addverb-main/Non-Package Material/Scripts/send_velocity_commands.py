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

    def stop_robot():
        rospy.loginfo("Stopping the robot...")
        zero_velocity = Float64(0.0)
        pub_joint1.publish(zero_velocity)
        pub_joint2.publish(zero_velocity)
        pub_joint3.publish(zero_velocity)
        pub_joint4.publish(zero_velocity)
        pub_joint5.publish(zero_velocity)
        pub_joint6.publish(zero_velocity)

    rospy.on_shutdown(stop_robot)

    rate = rospy.Rate(1000)  # 1000 Hz seems excessively high; consider lowering if not necessary

    while not rospy.is_shutdown():
        velocity = Float64(0.05)  # Set the desired velocity here
        rospy.loginfo("Sending velocity {} to all joints".format(velocity.data))
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
        rospy.loginfo("Program interrupted before completion.")
