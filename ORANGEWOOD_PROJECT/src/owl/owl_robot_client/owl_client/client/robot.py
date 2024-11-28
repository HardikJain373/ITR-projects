import logging
import socket

from owl_client.client.Interfaces import GRPClient, RobotMonitorSocket
from owl_client.client.Interfaces.utils import *

class OwlClient(object):
    """Class to act as client interface to interact with OWLRobot over a network(connection) recommended to use
    ethernet. OWLRobot running a GRPC server with a socket interface that make the robot state data available at 125Hz with no load.
    This client class initialise a grpc channel stub which can call the OWLRobot GRPC server functions. These GRPC calls are wrapped by 
    this class methods that can be called from OwlClient object. A socket listener is also implemented which
    listen to OWLRobot state socket interface and update the state data.
    """
    def __init__(self, host):
        self.logger = logging.getLogger("OWLRobot")
        self.logger.setLevel(logging.DEBUG)
        self.grpc_client_port = 18861
        self.monitor_interface_port = 30001
        self.host = host
        self.logger.debug("Opening robot monitor socket.....")
        self.robot_monitor = RobotMonitorSocket(self.host, self.monitor_interface_port)
        self.grpc_client = GRPClient(self.host, self.grpc_client_port)
        self.robot_monitor.start()
        self.logger.info("Started OWl Client.")

    def __repr__(self):
        return "Robot Object (IP=%s, state=%s)" % (self.host, self.__get_robot_mode())

    def __str__(self):
        return self.__repr__()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
    
    def is_running(self) -> bool:
        """
        Return True if robot is available to operate else False.
        """
        _mode = self.robot_monitor.getRobotMode(wait=True)
        return _mode != RobotMode.POWER_OFF and _mode != RobotMode.NOT_AVAILABLE

    def is_script_running(self) -> bool:
        """
        Return True if robot is running xml script program else False.
        """
        return self.robot_monitor.getProgramStatus(wait=True) == ProgramMode.RUNNING
    
    def get_version(self) -> str:
        """
        Return the current robot version.
        """
        return self.grpc_client.version()

    def get_joint(self, wait=True) -> Joint:
        """
        Return the current joint values of robot,wait will allow to return latest data recieved from robot.
        """
        return Joint(*self.robot_monitor.getQActual(wait))

    def get_tcp(self, wait=True) -> Pose:
        """
        Return the current tcp values of robot,wait will allow to return latest data recieved from robot.
        """
        return Pose(*self.robot_monitor.getTCPActual(wait))

    def get_robot_mode(self, wait=False) -> int:
        """
        Return the robot current operational mode, wait will allow to return latest data recieved from robot.
        """
        return self.robot_monitor.getRobotMode(wait)

    def get_program_mode(self , wait=False) -> int:
        """
        Return the robot current program execution mode, wait will allow to return latest data recieved from robot.
        """
        return self.robot_monitor.getProgramStatus(wait)

    def get_fk(self, joints : list):
        """
        Call robot server to solve forward kinematics request for given joint values.
        """
        return self.grpc_client.fk_request(joints)

    def get_ik(self, initial_guess : list, pose : list):
        """
        Call robot server to solve inverse kinematics request for given inital guess and pose.
        """
        return self.grpc_client.ik_request(initial_guess, pose)
    
    def set_payload_mass(self, mass : float):
        """
        Call robot server to set the payload mass (Kgs).
        """
        return self.grpc_client.payloadMass(mass)
    
    def set_payload_cog(self, cog : float):
        """
        Call robot server to set the payload centre  of mass (mm) from tool mount.
        """
        return self.grpc_client.payloadCOG(cog)
    
    def set_payload_inertia(self, inertia : list):
        """
        Call robot server to set the payload inertia tensor (mm) [ixx , ixy , ixz , iyy , iyz , izz].
        """
        return self.grpc_client.payloadInertia(inertia)
    
    def set_payload(self, mass : float, cog : float , inertia : list):
        """
        Call robot server to set the payload mass (Kgs).
        """
        return self.grpc_client.payload(mass, cog , inertia)
    
    def set_tcp_transform(self, transform : list):
        """
        Call robot server to set the tcp transform  [x(mm),y(mm),z(mm),rx(rad),ry(rad),rz(rad)].
        """
        return self.grpc_client.tcpTransform(transform)
    
    def set_gravity_vectory(self , g_vector : list):
        """
        Call robot server to set gravity vector [gravity_x, gravity_y , gravity_z].
        """
        return self.grpc_client.gravityVector(g_vector)

    def power_on(self):
        """
        Call robot server to power on.
        """
        return self.grpc_client.power_on()

    def power_down(self):
        """
        Call robot server to power down.
        """
        return self.grpc_client.power_down()

    def enter_teach_mode(self):
        """
        Call robot server to enter TeachMode.
        """
        return self.grpc_client.enter_teach_mode()

    def end_teach_mode(self):
        """
        Call robot server to end TeachMode.
        """
        return self.grpc_client.end_teach_mode()

    def set_digital_output(self, digital_pin: int, digital_status: bool):
        """
        Call robot server to set the level of digital output pin in PLC.
        """
        return self.grpc_client.set_digital_output(digital_pin, digital_status)

    def send_script(self, script: str):
        """
        Send a xml script program to robot to run.
        """
        self.grpc_client.send_script_program(script)

    def pause_script(self):
        """
        Pause a running script program
        """
        self.grpc_client.pause_script_program()

    def resume_script(self):
        """
        Resume a paused script program
        """
        self.grpc_client.resume_script_program()

    def stop_script(self):
        """
        Stop a script program
        """
        self.grpc_client.stop_script_program()

    def move_to_pose( self, goalPose: Pose, toolSpeed: float, wait=True, relative=False, moveType=TrajectoryPlanMode.STRAIGHT):
        """
        Request robot server to move to goal pose with desired tool speed in cartesian space.

        Parameters:
            goalPose (Pose)   : Goal pose [x,y,z,rx,ry,rz] robot need to achieve with desired tool speed.\n
            toolSpeed(float) : Tool speed with which robot need to do move.\n
            wait     (bool)      : True will make the move call synchronouse and wait till move is completed.\n
            relative (bool)    : Move relative to current robot pose.\n
            moveType (int)     : Type of move plan need to generate for move.\n
        """
        if type(goalPose) != Pose:
            raise Exception("Argument [goalPose] should be type of Pose")
        command = Pose()
        if relative:
            current_pose = self.get_tcp(wait=True)
            command.x = current_pose.x + goalPose.x
            command.y = current_pose.y + goalPose.y
            command.z = current_pose.z + goalPose.z
            command.roll = current_pose.roll + goalPose.roll
            command.pitch = current_pose.pitch + goalPose.pitch
            command.yaw = current_pose.yaw + goalPose.yaw
        else:
            command = goalPose
        return self.grpc_client.move_to_pose(command.get_pose(), toolSpeed, move_type=moveType, wait=wait)

    def move_to_joint(self, jointPose: Joint, toolSpeed: float, wait=True, relative=False):
        """
        Request robot server to move to goal joint with desired tool speed in joint space.

        Parameters:
            joalPose (Joint)   : Goal joint robot need to achieve with desired tool speed in radians.
            toolSpeed (float) : Tool speed with which robot need to do move.
            wait (bool)       : True will make the move call synchronouse and wait till move is completed.
            relative(bool)    : Move relative to current robot joint.
        """
        if type(jointPose) != Joint:
            raise Exception("Argument [jointPose] should be type of Joint")
        command = Joint()
        if relative:
            current_joint = self.get_joint(wait=True)
            command.Base = current_joint.Base + jointPose.Base
            command.Elbow = current_joint.Elbow + jointPose.Elbow
            command.Shoulder = current_joint.Shoulder + jointPose.Shoulder
            command.Wrist1 = current_joint.Wrist1 + jointPose.Wrist1
            command.Wrist2 = current_joint.Wrist2 + jointPose.Wrist2
            command.Wrist3 = current_joint.Wrist3 + jointPose.Wrist3
        else:
            command = jointPose
        return self.grpc_client.move_to_joint(command.get_joints(), toolSpeed, wait)

    def move_process(self, processWaypoints: list, toolSpeed: float, refrenceAcceleration: float = 0.005, wait=True):
        """
        Request robot server to do move process with desired tool speed.

        Parameters:
            processWaypoints (list)     : Cartesian poses to do a move process.\n
            toolSpeed (float)           : Tool speed with which robot need to do move.\n
            refrenceAcceleration(float) : Configure the blending radius in move process planning.\n
            wait (bool)                 : True will make the move call synchronouse and wait till move is completed.\n
        """
        return self.grpc_client.move_process(processWaypoints, toolSpeed, refrenceAcceleration, wait)

    def move_trajectory(self, trajectory : Trajectory):
        """
        Request robot server to follow a joint trajectory.

        Parameters:
            trajectory (Trajectory)     : Desired trajectory need to follow by robot.\n
        """
        if type(trajectory) != Trajectory:
            raise Exception("Argument [trajectory] should be type of Trajectory")
        return self.grpc_client.move_trajectory(trajectory)
    
    def move_translate(self, x=0.0, y=0.0, z=0.0, toolSpeed=100):
        """
        Request robot server to translate in cartesian space.

        Parameters:
            x(float) : Translate in x direction.\n
            y(float) : Translate in y direction.\n
            z(float) : Translate in z direction.\n
            toolSpeed (float) : Tool speed with which robot need to do move.\n
        """
        command = Pose()
        current_pose = self.get_tcp(wait=True)
        command.x = current_pose.x + x
        command.y = current_pose.y + y
        command.z = current_pose.z + z
        command.roll = current_pose.roll
        command.pitch = current_pose.pitch
        command.yaw = current_pose.yaw
        return self.grpc_client.move_to_pose(command.get_pose(), toolSpeed)

    def move_up(self, z=0.05, toolSpeed=100):
        """
        Request robot server to move in up.

        Parameters:
            z(float) : Translate in z direction.\n
            toolSpeed (float) : Tool speed with which robot need to do move.\n
        """
        command = Pose()
        current_pose = self.get_tcp(wait=True)
        command.x = current_pose.x
        command.y = current_pose.y
        command.z = current_pose.z + z
        command.roll = current_pose.roll
        command.pitch = current_pose.pitch
        command.yaw = current_pose.yaw
        return self.grpc_client.move_to_pose(command.get_pose(), toolSpeed)

    def move_down(self, z=-0.05, toolSpeed=100):
        """
        Request robot server to move in down.
        
        Parameters:
            z(float) : Translate in z direction.\n
            toolSpeed (float) : Tool speed with which robot need to do move.\n
        """
        return self.move_up(z , toolSpeed)

    def move_pause(self):
        """
        Request robot server to pause the current move.
        """
        return self.grpc_client.move_pause()

    def move_resume(self):
        """
        Request robot server to resume the paused move.
        """
        return self.grpc_client.move_resume()

    def move_abort(self):
        """
        Request robot server to abort the current move.
        """
        return self.grpc_client.move_arbot()

    def change_speed_fraction(self, speedFraction : float):
        """
        Change the speed fraction setting for move.
        """
        return self.grpc_client.set_speed_fraction(speedFraction)
    
    def update(self, access_code : str):
        """
        Request robot server to run update routine on robot. This command is admin privileged, an access code is required.
        
        Parameters:
            access_code (str): 
        """
        return self.grpc_client.system_update(access_code)
    
    def enable(self, access_code : str):
        """
        Request to enable the robot. This command is admin privileged, an access code is required.
        
        Parameters:
            access_code (str): 
        """
        return self.grpc_client.system_enable(access_code)
    
    def disable(self, access_code : str):
        """
        Request to disable the robot. This command is admin privileged, an access code is required.
        
        Parameters:
            access_code (str): 
        """
        return self.grpc_client.system_disable(access_code)
        
    def close(self):
        self.robot_monitor.close()
        self.grpc_client.close()
