from enum import IntEnum

class RobotMode(IntEnum):
   NOT_AVAILABLE = -1
   POWER_OFF = 0
   IDLE = 1
   RUNNING = 3
   PAUSED = 2
   TEACH_MODE = 5
   ERROR = -2