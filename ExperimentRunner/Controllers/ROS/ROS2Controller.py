from pathlib import Path
from ExperimentRunner.Controllers.ROS.IROSController import IROSController


class ROS2Controller(IROSController):
    def roslaunch_launch_file(self, launch_file: Path):
        pass
