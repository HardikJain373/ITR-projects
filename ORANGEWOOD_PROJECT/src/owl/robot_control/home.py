from owl_client import OwlClient, Joint, Pose
import time
import numpy as np

client = OwlClient("10.42.0.54")
jointSpeed = 30 #degrees/sec

# Wait for robot to be available
while not client.is_running():
    time.sleep(0.2)



joint = Joint()
joint.Base = 0
joint.Elbow = 0
joint.Shoulder = 0
joint.Wrist1 = 0
joint.Wrist2 = 0
joint.Wrist3 = 0

client.move_to_joint(joint, jointSpeed)
time.sleep(1)

