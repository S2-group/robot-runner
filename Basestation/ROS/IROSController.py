import os
import sys
import subprocess
from pathlib import Path
from abc import ABC, abstractmethod
from Procedures.ProcessProcedure import ProcessProcedure
from Procedures.OutputProcedure import OutputProcedure as output

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

    # Possibly need to kill _ros2_daemon every call!?
    def get_available_topics(self):
        command = "rostopic list" if ros_version == 1 else "ros2 topic list"
        return str(subprocess.check_output(command.split(' ')))

    # Possibly need to kill _ros2_daemon every call!?
    def get_available_nodes(self):
        command = "rosnode list" if ros_version == 1 else "ros2 node list"
        return str(subprocess.check_output(command.split(' ')))

    def are_nodes_available(self, node_names):
        nodes = self.get_available_nodes()
        all_available = False
        for node in node_names:
            all_available = node in nodes

        return all_available

    def are_topics_available(self, topic_names):
        topics = self.get_available_topics()
        all_available = False
        for topic in topic_names:
            all_available = topic in topics

        return all_available

    @abstractmethod
    def roscore_start(self):
        pass

    @abstractmethod
    def roslaunch_launch_file(self, launch_file: Path):
        pass

    @abstractmethod
    def rosbag_start_recording_topics(self, topics, file_path: str, bag_name):
        pass

    @abstractmethod
    def rosbag_stop_recording_topics(self, bag_name):
        pass

    @abstractmethod
    def sim_shutdown(self):
        pass

    @abstractmethod
    def native_run_end(self):
        pass
