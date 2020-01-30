from pathlib import Path
from abc import ABC, abstractmethod


class IROSController(ABC):
    roslaunch_proc = None
    roslaunch_pid: str = "/tmp/roslaunch.pid"

    @abstractmethod
    def get_gazebo_time(self):
        pass

    @abstractmethod
    def roslaunch_launch_file(self, launch_file: Path):
        pass

    @abstractmethod
    def ros_shutdown(self):
        pass

    @abstractmethod
    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        pass

    @abstractmethod
    def rosbag_stop_recording_topics(self, bag_name):
        pass
