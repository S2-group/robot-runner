import os
import sys
import time
import rospy
import signal
import subprocess
from pathlib import Path
from roslaunch import RLException
from ExperimentRunner.Utilities.Utils import Utils
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ROS1Controller(IROSController):
    FNULL = Utils.FNULL
    roslaunch_proc = None
    roslaunch_pid: str = "/tmp/roslaunch.pid"

    def roslaunch_launch_file(self, launch_file: Path):  # TODO: add log_file to output roslaunch pipe to
        output.console_log(f"Roslaunch {launch_file}")
        command = f"roslaunch --pid=/tmp/roslaunch.pid {launch_file}"
        try:
            self.roslaunch_proc = subprocess.Popen(command, shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)
            time.sleep(1)
        except RLException:
            output.console_log("Something went wrong launching the launch file.")
            sys.exit(1)

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        output.console_log(f"Rosbag starts recording...")  # TODO: output what topics are recorded

        # Build 'rosbag record -O filename [/topic1 /topic2 ...] __name:=bag_name' command
        command = f"rosbag record -O {file_path}"
        for topic in topics:
            command += f" {topic}"
        command += f" __name:={bag_name}"

        try:
            subprocess.Popen(command, shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)
            time.sleep(1)  # Give rosbag recording some time to initiate
        except RLException:
            output.console_log("Something went wrong recording topics to rosbag")
            sys.exit(1)

    def rosbag_stop_recording_topics(self, bag_name):
        output.console_log(f"Stop recording rosbag on ROS node: {bag_name}")
        subprocess.call(f"rosnode kill {bag_name}", shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)

    def ros_shutdown(self):
        output.console_log("Terminating roslaunch launch file...")
        self.roslaunch_proc.send_signal(signal.SIGINT)
        try:
            with open("/tmp/roslaunch.pid", 'r') as myfile:
                pid = int(myfile.read())
                os.kill(pid, signal.SIGINT)
        except FileNotFoundError:
            output.console_log("roslaunch pid not found, continuing normally...")

        while self.roslaunch_proc.poll() is None:
            output.console_log_animated("Waiting for graceful exit...")

        rospy.signal_shutdown("Run completed")
        output.console_log("Roslaunch launch file successfully terminated!")

