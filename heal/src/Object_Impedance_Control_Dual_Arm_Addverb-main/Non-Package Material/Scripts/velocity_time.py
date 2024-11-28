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

    # List of (velocity, joint_publisher, duration) tuples
    commands = [
        (1, pub_joint1, 0.5),  # Give joint 1 a velocity of 0.1 for 2 seconds
        (0.2, pub_joint2, 0.5),  # Give joint 2 a velocity of 0.2 for 3 seconds
        # Add more tuples as needed
    ]

    def stop_robot():
        rospy.loginfo("Stopping the robot...")
        zero_velocity = Float64(0.0)
        for pub in [pub_joint1, pub_joint2, pub_joint3, pub_joint4, pub_joint5, pub_joint6]:
            pub.publish(zero_velocity)

    rospy.on_shutdown(stop_robot)

    for velocity, joint_pub, duration in commands:
        rospy.loginfo(f"Sending velocity {velocity} to joint for {duration} seconds")
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)  # 100 Hz rate

        # Publish velocity until the duration is met
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            joint_pub.publish(Float64(velocity))
            rate.sleep()

    stop_robot()  # Ensure the robot is stopped at the end of all commands

if __name__ == '__main__':
    try:
        send_velocity_commands()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion.")


##https://answers.ros.org/question/293830/what-is-the-fix-for-no-p-gain-specified-for-pid-namespace-gazebo_ros_controlpid_gainsback_right_wheel_joint-ros-melodic/