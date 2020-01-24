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
    def roscore(self):
        pass

    @abstractmethod
    def rosinit_robot_runner_node(self):
        pass

    @abstractmethod
    def ros_shutdown(self):
        pass
