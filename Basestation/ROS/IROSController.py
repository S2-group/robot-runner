import os
import sys
import subprocess
from pathlib import Path
from abc import ABC, abstractmethod
from Common.Procedures.ProcessProcedure import ProcessProcedure
from Common.Procedures.OutputProcedure import OutputProcedure as output

try:
    ros_version = int(os.environ['ROS_VERSION'])
except ValueError:
    output.console_log_bold("Unknown value for $ROS_VERSION env variable")
    sys.exit(1)


###     =========================================================
###     |                                                       |
###     |                     IROSController                    |
###     |       - Provide abstract, implementation specific     |
###     |         methods for ROS1 or ROS2 to implement         |
###     |                                                       |
###     |       - Provide default, generic functions for both   |
###     |         ROS versions like get_available_topics()      |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific (ROS1 or ROS2) should be declared    |
###     |         here as an abstract function                  |
###     |                                                       |
###     |       * Any generic functionality between the two     |
###     |         ROS types should be declared here             |
###     |         as a function                                 |
###     |                                                       |
###     =========================================================
class IROSController(ABC):
    roslaunch_proc = None
    roscore_proc = None

    @abstractmethod
    def rosbag_start_recording_topics(self, topics, file_path: str, bag_name):
        pass

    @abstractmethod
    def rosbag_stop_recording_topics(self, bag_name):
        pass