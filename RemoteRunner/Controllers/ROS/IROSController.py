import os
import sys
import subprocess
from pathlib import Path
from abc import ABC, abstractmethod


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

    @abstractmethod
    def get_available_topics(self):
        pass

    @abstractmethod
    def get_available_nodes(self):
        pass

    @abstractmethod
    def are_topics_available(self, topic_names):
        pass

    @abstractmethod
    def are_nodes_available(self, node_names):
        pass

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
