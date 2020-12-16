from enum import Enum

class RobotRunnerEvents(Enum):
    BEFORE_EXPERIMENT = 1
    BEFORE_RUN = 2
    START_RUN = 3
    START_MEASUREMENT = 4
    LAUNCH_MISSION = 5
    CONTINUE = 6
    STOP_MEASUREMENT = 7
    STOP_RUN = 8
    POPULATE_RUN_DATA = 9
    AFTER_EXPERIMENT = 10