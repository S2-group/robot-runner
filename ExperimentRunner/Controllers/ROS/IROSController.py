from pathlib import Path
from abc import ABC, abstractmethod


class IROSController(ABC):
    @abstractmethod
    def roslaunch_launch_file(self, launch_file: Path):
        pass

    @abstractmethod
    def subscribe_to_topic(self, topic, datatype, callback):
        pass

    @abstractmethod
    def roscore_start(self):
        pass

    @abstractmethod
    def rosinit_robot_runner_node(self):
        pass

    @abstractmethod
    def ros_shutdown(self):
        pass

    @abstractmethod
    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        pass

    @abstractmethod
    def rosbag_stop_recording_topics(self, bag):
        pass
