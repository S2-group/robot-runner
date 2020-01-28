from pathlib import Path
from ExperimentRunner.Controllers.ROS.IROSController import IROSController


class ROS2Controller(IROSController):
    def roslaunch_launch_file(self, launch_file: Path):
        pass

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        pass

    def rosbag_stop_recording_topics(self, bag):
        pass

    def ros_shutdown(self):
        pass