import subprocess
from subprocess import CalledProcessError


class RobotClient:
    roscore_node_name: str

    def __init__(self):
        self.roscore_node_name = "/rosout"
        while not self.is_roscore_ready():
            print("Waiting for ROS Master...", end="\r")

        print("ROS Master ready!")

    def is_roscore_ready(self):
        try:
            available_nodes = str(subprocess.check_output('rosnode list', shell=True))
            return self.roscore_node_name in available_nodes
        except CalledProcessError:
            print("rosnode list failed, ROS Master (roscore) seems unavailable")
            return False

    def roslaunch_launch_file(self):
        pass  # TODO: Roslaunch either ROS1 or ROS2

    def run_wait_completed(self):
        pass

    def run_end(self):
        pass


if __name__ == "__main__":
    RobotClient()
