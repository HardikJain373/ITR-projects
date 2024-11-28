#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

def control_joints_to_target_positions():
    rospy.init_node('joint_position_controller')

    # Publishers for each joint's velocity command
    velocity_pubs = [
        rospy.Publisher('/joint{}_velocity_controller/command'.format(i+1), Float64, queue_size=10)
        for i in range(6)
    ]

    # Subscribe to joint states to get current positions
    current_positions = [0.0] * 6
    last_errors = [0.0] * 6  # To store last position error for derivative calculation
    def joint_states_callback(msg):
        joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for name, pos in zip(msg.name, msg.position):
            if name in joint_order:
                index = joint_order.index(name)
                current_positions[index] = pos
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # Define target positions and gains
    target_positions = [1.5, 0.5, 0.0, 0, 0, 0]  # Example target positions in radians
    kp = 0.1  # Proportional gain
    kd = 0.01  # Derivative gain

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        for i in range(6):
            position_error = target_positions[i] - current_positions[i]
            # Calculate derivative of error
            error_derivative = position_error - last_errors[i]
            last_errors[i] = position_error

            velocity_command = kp * position_error + kd * error_derivative
            velocity_pubs[i].publish(velocity_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_joints_to_target_positions()
    except rospy.ROSInterruptException:
        pass
