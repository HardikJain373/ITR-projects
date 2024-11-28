class Trajectory(object):
    """Represent trajectory, which have positions, velocities and acceleration profiles."""
    positions = []
    velocities = []
    accelerations = []
    
    def __init__(self, positions=[], velocities=[], accelerations=[]) -> None:
        self.positions = positions
        self.velocities = velocities
        self.accelerations = accelerations
        
    def getPositions(self) -> list:
        return self.positions
    
    def getVelocities(self) -> list:
        return self.velocities
    
    def getAccelerations(self) -> list:
        return self.accelerations
