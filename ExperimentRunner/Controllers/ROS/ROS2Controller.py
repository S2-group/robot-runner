import os
import sys
import time
import signal
import subprocess
from pathlib import Path
from importlib import import_module
from ExperimentRunner.Utilities.Utils import Utils
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ROS2Controller(IROSController):
    def get_gazebo_time(self):
        return int(str(subprocess.check_output("ros2 topic echo -n 1 /clock", shell=True)).split('secs: ')[1].split('\\n')[0])

    def __init__(self):
        self.rclpy = import_module('rclpy')

    def roslaunch_launch_file(self, launch_file: Path):
        output.console_log(f"ros2 launch {launch_file}")
        command = f"ros2 launch {launch_file}"
        try:
            self.roslaunch_proc = subprocess.Popen(command, shell=True, stdout=Utils.FNULL, stderr=subprocess.STDOUT)
            time.sleep(1)
        except:
            output.console_log("Something went wrong launching the launch file.")
            sys.exit(1)

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        file_path += "-ros1"
        output.console_log(f"Rosbag2 starts recording...")
        output.console_log_bold("Recording topics: ")
        # Build 'rosbag record -O filename [/topic1 /topic2 ...] __name:=bag_name' command
        command = f"ros2 bag record -O {file_path}"
        for topic in topics:
            command += f" {topic}"
            output.console_log_bold(f" * {topic}")
        command += f" __name:={bag_name}"

        try:
            subprocess.Popen(command, shell=True, stdout=Utils.FNULL, stderr=subprocess.STDOUT)
            time.sleep(1)  # Give rosbag recording some time to initiate
        except:
            output.console_log("Something went wrong recording topics to rosbag")
            sys.exit(1)

    def rosbag_stop_recording_topics(self, bag_name):
        output.console_log(f"Stop recording rosbag on ROS node: {bag_name}")
        subprocess.call(f"ros2 lifecycle set {bag_name} shutdown", shell=True, stdout=Utils.FNULL, stderr=subprocess.STDOUT)

    def ros_shutdown(self):
        output.console_log("Terminating roslaunch launch file...")
        subprocess.call("ros2 service call /gazebo/reset_simulation \"{}\"", shell=True)

        self.roslaunch_proc.send_signal(signal.SIGINT)
        try:
            pid = self.roslaunch_proc.pid
            os.kill(pid, signal.SIGINT)
        except:
            output.console_log("Terminating process by PID unsuccessful")

        while self.roslaunch_proc.poll() is None:
            output.console_log_animated("Waiting for graceful exit...")

        self.rclpy.shutdown()
        output.console_log("Roslaunch launch file successfully terminated!")