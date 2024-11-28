from owl_client import OwlClient, Joint, Pose
import time
import numpy as np

client = OwlClient("10.42.0.54")
toolSpeed = 35 

# Wait for robot to be available
while not client.is_running():
    time.sleep(1)

def create_circle_positions(initial_position, circle_radius):
    circle_positions = []
    theta = 0
    d_theta = np.pi/8
    position_x, position_y = initial_position[0], initial_position[1]
    while theta < 2*np.pi:
        position_x += circle_radius * np.cos(theta) * d_theta
        position_y += circle_radius * np.sin(theta) * d_theta
        circle_positions.append([position_x, position_y, initial_position[2], initial_position[3], initial_position[4], initial_position[5]])
        theta += d_theta

    return circle_positions

def create_rectange_positions(initial_position, rectange_width, rectange_height):
    rectange_positions = []
    position_x, position_y = initial_position[0], initial_position[1]
    rectange_positions.append([position_x, position_y, initial_position[2], initial_position[3], initial_position[4], initial_position[5]])
    position_x += rectange_width
    rectange_positions.append([position_x, position_y, initial_position[2], initial_position[3], initial_position[4], initial_position[5]])
    position_y += rectange_height
    rectange_positions.append([position_x, position_y, initial_position[2], initial_position[3], initial_position[4], initial_position[5]])
    position_x -= rectange_width
    rectange_positions.append([position_x, position_y, initial_position[2], initial_position[3], initial_position[4], initial_position[5]])
    position_y -= rectange_height
    rectange_positions.append([position_x, position_y, initial_position[2], initial_position[3], initial_position[4], initial_position[5]])

    return rectange_positions



#create joint goals for robot

initial_x = 0.6
initial_y = 0.006
initial_z = 0.42
radius = 0.2

position_1 =[0.6, 0.006, 0.42, -0.28, 0.0, -1.56] # Start position
circle_positions = create_circle_positions(position_1, radius) # Circle positions
position_2 = [0.6, 0.006, 0.47, 0.0, 0.0, 0.0] # Circle end position
position_3 = [0.6, 0.106, 0.47, 0.0, 0.0, 0.0] # Rectangle start position
position_4 = [0.6, 0.106, 0.42, 0, 0, 0]
rectange_positions = create_rectange_positions(position_4, radius*2, radius*2) # Rectangle positions
position_5 = [0.6, 0.006, 0.52, 0.0, 0.0, 0.0] # End position


pos_arr = [position_1] + [position for position in rectange_positions] + [position_2] + [position_3] + [position_4] + [position for position in rectange_positions]


for pos in pos_arr:
    pose = Pose()
    pose.x = pos[0]
    pose.y = pos[1]
    pose.z = pos[2]
    pose.roll = pos[3]
    pose.pitch = pos[4]
    pose.yaw = pos[5]
     
    client.move_to_pose(pose, toolSpeed)
    time.sleep(1)
