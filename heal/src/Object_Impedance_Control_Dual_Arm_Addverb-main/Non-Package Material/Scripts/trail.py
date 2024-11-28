import rospy
from std_msgs.msg import Float64
import threading
import numpy as np
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import Chain, ChainIkSolverPos_LMA, Frame, Rotation, Vector, JntArray
from gazebo_msgs.srv import GetModelState
from scipy.spatial.transform import Rotation as R

# Global publisher dictionary
publishers = {}

def setup_publishers(namespaces, joint_names):
    """Pre-initialize publishers for all joints and namespaces."""
    for namespace in namespaces:
        for name in joint_names:
            topic = f'/{namespace}/{name}_position_controller/command'
            publishers[(namespace, name)] = rospy.Publisher(topic, Float64, queue_size=10)
    rospy.sleep(1)  # Allow time for publishers to set up

def load_robot_model(namespace):
    """ Load the URDF from the parameter server and construct the KDL tree """
    param_name = f'{namespace}/robot_description'
    try:
        robot_urdf = URDF.from_parameter_server(key=param_name)
        success, kdl_tree = treeFromUrdfModel(robot_urdf)
        if success:
            rospy.loginfo(f"KDL tree for {namespace} created successfully.")
            # Assuming you know the base link and tip link names
            base_link = 'base_link'  # Replace with your actual base link name
            tip_link = 'end_effector'  # Replace with your actual tip link name
            kdl_chain = kdl_tree.getChain(base_link, tip_link)
            return kdl_chain
        else:
            rospy.logerr(f"Failed to construct KDL tree for {namespace}.")
    except Exception as e:
        rospy.logerr(f"Error loading URDF from parameter server for {namespace}: {e}")
    return None

def compute_ik(kdl_chain, x, y, z, roll, pitch, yaw):
    """ Compute inverse kinematics using KDL """
    ik_solver = ChainIkSolverPos_LMA(kdl_chain)
    target_frame = Frame(Rotation.RPY(roll, pitch, yaw), Vector(x, y, z))
    joint_positions = JntArray(kdl_chain.getNrOfJoints())
    if ik_solver.CartToJnt(joint_positions, target_frame, joint_positions) >= 0:
        return joint_positions
    else:
        rospy.logerr("IK computation failed.")
        return None

def move_joints_once(namespace, target_pose):
    """ Calculate joint positions using IK and publish them """
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    kdl_chain = load_robot_model(namespace)
    if kdl_chain:
        joint_positions = compute_ik(kdl_chain, *target_pose)
        if joint_positions:
            for i, name in enumerate(joint_names):
                position = Float64(joint_positions[i])
                rospy.loginfo(f"Publishing {position.data} to {namespace}: {name}")
                publishers[(namespace, name)].publish(position)

def start_thread(namespace, target_pose):
    """ Start a thread to move joints """
    thread = threading.Thread(target=move_joints_once, args=(namespace, target_pose))
    thread.start()
    return thread

def get_box_pose(model_name='my_box'):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state(model_name, "")
        
        box_position = response.pose.position
        box_orientation = response.pose.orientation
        
        box_pose = (
            box_position.x, 
            box_position.y, 
            box_position.z, 
            box_orientation.x, 
            box_orientation.y, 
            box_orientation.z, 
            box_orientation.w
        )
        
        rospy.loginfo(f"Box pose: {box_pose}")
        return box_pose
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def compute_target_pose(box_pose, box_dimensions, robot_offset=0.0, is_robot1=True):
    x, y, z, qx, qy, qz, qw = box_pose
    length, width, height = box_dimensions

    # Convert quaternion to a rotation matrix
    rotation = R.from_quat([qx, qy, qz, qw])

    # Define the offset vector in the box's local frame (aligned with its length)
    local_offset = np.array([length / 2.0 + robot_offset, 0, 0]) * (-1 if not is_robot1 else 1)
    local_offset[0] += 0.5 if not is_robot1 else 0  # Adjust for Robot 2's position if necessary

    # Rotate the local offset to align with the box's global orientation
    global_offset = rotation.apply(local_offset)
    target_position = np.array([x, y, z]) + global_offset

    # Adjust orientation for end-effector to face towards or away from the box center
    target_orientation = rotation.as_euler('xyz', degrees=False)
    target_orientation[2] += np.pi if not is_robot1 else 0

    return (*target_position, *target_orientation)

def main():
    rospy.init_node('joint_position_publisher_once', anonymous=True)

    # Get the box pose from the Gazebo service
    box_pose = get_box_pose(model_name='my_box')
    if box_pose is None:
        rospy.logerr("Failed to get the box pose. Exiting...")
        return

    box_dimensions = (0.25, 0.2, 0.15)  # length, width, height
    robot_offset = 0.00  # 5 cm offset

    # Compute target poses for both robots
    target_pose_robot1 = compute_target_pose(box_pose, box_dimensions, robot_offset, is_robot1=True)
    target_pose_robot2 = compute_target_pose(box_pose, box_dimensions, robot_offset, is_robot1=False)

    rospy.loginfo(f"Robot 1 Target Pose: {target_pose_robot1}")
    rospy.loginfo(f"Robot 2 Target Pose: {target_pose_robot2}")

    namespaces = ['robot1', 'robot2']
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    setup_publishers(namespaces, joint_names)

    # Start sending
    # joint positions to both robots simultaneously
    thread1 = start_thread('robot1', target_pose_robot1)
    thread2 = start_thread('robot2', target_pose_robot2)

    # Wait for both threads to complete
    thread1.join()
    thread2.join()

    rospy.spin()  # Keep the node alive if necessary

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass