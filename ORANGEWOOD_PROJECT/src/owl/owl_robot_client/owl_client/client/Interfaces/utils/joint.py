class Joint(object):
    """Represent joints configuration in radians."""

    Base = 0.0
    Shoulder = 0.0
    Elbow = 0.0
    Wrist1 = 0.0
    Wrist2 = 0.0
    Wrist3 = 0.0

    def __init__(self, *args, **kwargs) -> None:
        if len(args) == 6:
            (
                self.Base,
                self.Shoulder,
                self.Elbow,
                self.Wrist1,
                self.Wrist2,
                self.Wrist3,
            ) = args
        else:
            self.Base = kwargs.get("Base", 0.0)
            self.Shoulder = kwargs.get("Shoulder", 0.0)
            self.Elbow = kwargs.get("Elbow", 0.0)
            self.Wrist1 = kwargs.get("Wrist1", 0.0)
            self.Wrist2 = kwargs.get("Wrist2", 0.0)
            self.Wrist3 = kwargs.get("Wrist3", 0.0)

    def get_joints(self) -> list:
        return [
            self.Base,
            self.Shoulder,
            self.Elbow,
            self.Wrist1,
            self.Wrist2,
            self.Wrist3,
        ]

    def is_empty(self):
        return all(val == 0 for val in self.get_joint())

    def from_array(self, joint_array: list):
        (
            self.Base,
            self.Shoulder,
            self.Elbow,
            self.Wrist1,
            self.Wrist2,
            self.Wrist3,
        ) = joint_array
