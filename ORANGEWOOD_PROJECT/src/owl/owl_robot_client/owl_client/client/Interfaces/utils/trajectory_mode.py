from enum import IntEnum

class TrajectoryPlanMode(IntEnum):
    STRAIGHT = 1
    ARC = 2
    CIRCLE = 3
    BSPLINE = 4