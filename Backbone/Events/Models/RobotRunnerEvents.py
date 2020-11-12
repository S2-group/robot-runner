from enum import Enum

class RobotRunnerEvents(Enum):
    BEFORE_EXPERIMENT = 1
    START_RUN = 2
    START_MEASUREMENT = 3
    #DURING_RUN = 4 # UNSUPPORTED FOR NOW (multiprocessing fails to find RobotRunnerConfig)
    SIGNAL_SEMI_CONTINUE = 5
    STOP_MEASUREMENT = 6
    STOP_RUN = 7
    POPULATE_RUN_DATA = 8
    AFTER_EXPERIMENT = 9