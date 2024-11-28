class Pose(object):
    """Represent cartesian pose, where x , y and z are in metres and roll, pitch
    and yaw are in radians.
    """

    x = 0.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    def __init__(self, *args, **kwargs) -> None:
        if len(args) == 6:
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw = args
        else:
            self.x = kwargs.get("x", 0.0)
            self.y = kwargs.get("y", 0.0)
            self.z = kwargs.get("z", 0.0)
            self.roll = kwargs.get("roll", 0.0)
            self.pitch = kwargs.get("pitch", 0.0)
            self.yaw = kwargs.get("yaw", 0.0)

    def get_pose(self) -> list:
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]

    def is_empty(self):
        return all(val == 0 for val in self.get_pose())

    def from_array(self, pose_array: list):
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = pose_array