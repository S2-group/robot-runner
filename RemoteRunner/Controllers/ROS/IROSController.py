import os
import sys
import subprocess
from pathlib import Path
from abc import ABC, abstractmethod
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output

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
    sim_poll_proc = None
    roslaunch_proc = None
    roscore_proc = None

    def is_gazebo_running(self):
        if not self.sim_poll_proc:
            dir_path = os.path.dirname(os.path.realpath(__file__)) + '/../Experiment/Run/Scripts'
            self.sim_poll_proc = subprocess.Popen(f"{sys.executable} {dir_path}/PollSimRunning.py", shell=True)

        # TODO: check return code if non-zero (error)
        return self.sim_poll_proc.poll() is not None

    def get_available_topics(self):
        command = "rostopic" if ros_version == 1 else "ros2 topic"
        return str(subprocess.check_output(f"{command} list", shell=True))

    def get_available_nodes(self):
        command = "rosnode" if ros_version == 1 else "ros2 node"
        return str(subprocess.check_output(f"{command} list", shell=True))

    def are_nodes_available(self, node_names):
        nodes = self.get_available_nodes()
        all_available = True
        for node in node_names:
            all_available = node in nodes

        return all_available

    def are_topics_available(self, topic_names):
        topics = self.get_available_topics()
        all_available = True
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
    def native_shutdown(self):
        pass
