# OWL Client

A python module to operate the OWLRobot on LAN. OWLRobot running a GRPC server with a socket interface that make the robot state data available at 125Hz with no load.

## Usage #
To get started, you can follow the below steps to start using the owlClient :

1. Install the module by running the following command in terminal where setup.py located.

    ```
        pip3 install -e .
    ```

2. First step of using any network communication interface is to make sure the client and host machine is able to find themselve over the network. The owlBot is running the serve on its all available ip address. Proceeding further find the ip address of the owlBot and setup a communication between the client and owlBot machine over ethernet.

3. To make sure the owlBot machine is reachable ping owlBot-IP, use following command :

    ```
        ping <owl-robot-ip>
    ```

4. Once, network communication setup is done, import the OwlClient start using it. Here is an example of getting current joints.

    ```python
    from owl_client import OwlClient
    import time

    _robot_ip = "<owl-robot-ip>"

    client = OwlClient(_robot_ip)

    # Wait for Robot to be available to operate
    while not client.is_running():
        time.sleep(0.2)

    while True:
        print("Current joint pose", client.get_joint().get_joints())
        time.sleep(0.01)
    ```

## API Guide #

Refer the for API documentation [here](https://owldoc.bitbucket.io/)

We can divide the API Guide into two section. First section deal with how to get the Robot relate data to implement logical decisions. Second section inculde the way to command the Robot.

### Robot State Data 

The OWLRobot is brodcasting a TCP/IP packet on socket (port 30001) at 125 Hz rate with no load condition which contain the following data :

```yaml
    TimeStamp
    RobotJointData [6]
    RobotTcpData  [6]
    RobotIOStatus #NA
    ProgramMode
    ProgramActiveStateID
    ProgramActiveAssignment
    RobotMode   
    RobotCode
    RobotExceptionSource
    RobotExceptionMessage
    OperatorRequestType
    OperatorRequestMessage
```
All these robot data is available through corresponding API calls.

To get the mode of the Robot you can call `get_robot_mode()` using the client. The following is the RobotModes enum which represent corresponding Robot operational mode.

```python
class RobotMode(IntEnum):
   NOT_AVAILABLE = -1
   POWER_OFF = 0
   IDLE = 1
   RUNNING = 3
   PAUSED = 2
   TEACH_MODE = 5
   ERROR = -2
```

Before sending motion command to the Robot you can make sure the Robot is in IDLE mode or you can call `is_running()` to check whether robot is available to operate or not. 

### Robot Motion Command

There are three type of API Motion command which the Robot will response.

1. MoveJ : Joint Command given to Robot with a given goal Joint. It will plan and do the  execution in joint space.
2. MoveL : Cartesian Command given to Robot with a given goal Pose. It will plan and do the execution in cartesian space.
3. MoveP : Process command given to Robot with list of cartesian poses with blend radiis to do a motion with a constant specified tool speed in mm/sec.

Here is a example of sync motion cartesian command given to robot.

```python
    from owl_client import OwlClient, Pose , RobotMode
    import time

    _robot_ip = "localhost"
    client = OwlClient(_robot_ip)

    # Wait for Robot to be available to operate
    while not client.is_running():
        time.sleep(0.2)

    # Just to make sure Robot is not busy
    if client.get_robot_mode() == RobotMode.IDLE
        _toolspeed = 100  # mm/sec
        # Pose 
        _pose1 = Pose()
        _pose1.x = -0.176
        _pose1.y = -0.240204
        _pose1.z = 0.489203
        _pose1.roll = 3.1376
        _pose1.pitch = -0.087288
        _pose1.yaw = 1.56449
        # Move to Pose
        client.move_to_pose(_pose1, _toolspeed)
```
You can parallely stop the motion command by calling `move_abort()`.

Refer `examples` folder for more example scripts.

