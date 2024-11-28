import grpc
import logging

import owl_client.client.Interfaces.grpc_client.robot_service_pb2 as robotServicePb2
import owl_client.client.Interfaces.grpc_client.robot_service_pb2_grpc as robotServiceGRPPb2
from   owl_client.client.Interfaces.utils import TrajectoryPlanMode

class GRPClient(object):
    def __init__(self , host , port):
        self.host = host
        self.server_port = port
        self.logger = logging.getLogger(self.__class__.__name__)
        self.channel = grpc.insecure_channel('{}:{}'.format(self.host, self.server_port))
        self.stub = robotServiceGRPPb2.RobotServiceStub(self.channel)
        
    def payloadMass(self, mass : float):
        _request = robotServicePb2.SetPayloadMassRequest()
        _request.mass = mass
        return self.stub.SetPayloadMass(_request)
    
    def payloadCOG(self, cog : list):
        _request = robotServicePb2.SetPayloadCOGRequest()
        _request.cog.extend(cog)
        return self.stub.SetPayloadCOG(_request)
    
    def payloadInertia(self, inertia : list):
        _request = robotServicePb2.SetPayloadInertiaMatrixRequest()
        _request.inertia.extend(inertia)
        return self.stub.SetPayloadInertiaMatrix(_request)
    
    def payload(self, mass : float, cog :list, inertia : list):
        _request = robotServicePb2.SetPayloadRequest()
        _request.mass = mass
        _request.cog.extend(cog)
        _request.inertia.extend(inertia)
        return self.stub.SetPayload(_request)

    def tcpTransform(self, transform : list):
        _request = robotServicePb2.SetTCPRequest()
        _request.transform.extend(transform)
        return self.stub.SetTCP(_request)

    def gravityVector(self, vector : list):
        _request = robotServicePb2.SetGravityVectorRequest()
        _request.vector.extend(vector)
        return self.stub.SetGravityVector(_request)
    
    def joggingFrame(self, frame : int):
        _request = robotServicePb2.SetCartesianJoggingFrameRequest()
        _request.type = frame
        return self.stub.SetCartesianJoggingFrame(_request)
            
    def power_on(self):
        return self.stub.PowerOn(robotServicePb2.PowerOnRequest())
    
    def power_down(self):
        return self.stub.PowerOn(robotServicePb2.PowerDownRequest())
    
    def enter_teach_mode(self):
        return self.stub.EnterTeachMode(robotServicePb2.EnterTeachModeRequest())
    
    def end_teach_mode(self):
        return self.stub.ExitTeachMode(robotServicePb2.ExitTeachModeRequest())
    
    def fk_request(self, joints):
        _request = robotServicePb2.FKRequestRequest()
        _request.joints.extend(joints)
        response = self.stub.FKRequest(_request)
        return response.pose
    
    def ik_request(self , inital_guess , pose):
        _request = robotServicePb2.IKRequestRequest()
        _request.initial_guess.extend(inital_guess)
        _request.pose.extend(pose)
        response = self.stub.IKRequest(_request)
        return response.joints
        
    def set_digital_output(self , pin , state):
        _request = robotServicePb2.SetDigitalOutputRequest()
        _request.output_pin = pin
        _request.pin_status = state
        return self.stub.SetDigitalOutput(_request)
    
    def set_speed_fraction(self , speed_fraction):
        return self.stub.SetSpeedFraction(robotServicePb2.SetSpeedFractionRequest(speed_fraction=speed_fraction))
    
    def system_update(self, access_code):
         return self.stub.SystemUpdate(robotServicePb2.SystemUpdateRequest(accessCode=access_code))
     
    def system_enable(self, access_code):
         return self.stub.SystemEnable(robotServicePb2.SystemEnableRequest(accessCode=access_code))
     
    def system_disable(self, access_code):
         return self.stub.SystemDisable(robotServicePb2.SystemDisableRequest(accessCode=access_code))
    
    def send_script_program(self, script):
        _request = robotServicePb2.SendScriptRequest()
        _request.program = script
        return self.stub.SendScript(_request)
    
    def pause_script_program(self):
        return self.stub.PauseScript(robotServicePb2.PauseScriptRequest())
    
    def resume_script_program(self):
        return self.stub.ResumeScript(robotServicePb2.ResumeScriptRequest())
    
    def stop_script_program(self):
        return self.stub.StopScript(robotServicePb2.StopScriptRequest())
    
    def move_to_pose(self , pose , tool_speed , move_type=TrajectoryPlanMode.STRAIGHT , wait=True):
        _request = robotServicePb2.MoveToPoseRequest()
        _request.pose.extend(pose)
        _request.tool_speed = tool_speed
        _request.move_type = move_type.value
        _request.wait = wait
        return self.stub.MoveToPose(_request)
    
    def move_to_joint(self , joints , joint_speed , wait=True):
        _request = robotServicePb2.MoveToJointRequest()
        _request.joints.extend(joints)
        _request.joint_speed = joint_speed
        _request.wait = wait
        return self.stub.MoveToJoint(_request)
    
    def move_process(self, pose_array , tool_speed , reference_acceleration=0.005 , wait=True):
       raise NotImplementedError()
    
    def move_trajectory(self, trajectoryGoal):
        positions = robotServicePb2.MultiArray()
        velocities = robotServicePb2.MultiArray()
        accelerations = robotServicePb2.MultiArray()
        
        for i in range(len(trajectoryGoal.positions)):
            position_message = positions.arrays.add()
            velocity_message = velocities.arrays.add()
            acceleration_message = accelerations.arrays.add()
            
            position_message.data.extend(trajectoryGoal.positions[i])
            velocity_message.data.extend(trajectoryGoal.velocities[i])
            acceleration_message.data.extend(trajectoryGoal.accelerations[i])
            
        _request = robotServicePb2.MoveTrajectoryRequest()
        _request.joint_points = positions.SerializeToString()
        _request.velocity_points = velocities.SerializeToString()
        _request.acceleration_points = accelerations.SerializeToString()
        return  self.stub.MoveTrajectory(_request)
    
    def move_pause(self):
        return self.stub.MovePause(robotServicePb2.MovePauseRequest())
        
    def move_resume(self):
        return self.stub.MoveResume(robotServicePb2.MoveResumeRequest())
    
    def move_arbot(self):
        return self.stub.MoveAbort(robotServicePb2.MoveAbortRequest())
    
    def version(self):
        return self.stub.GetVersion(robotServicePb2.GetVersionRequest()).version
    
    def close(self):
        self.channel.close()
