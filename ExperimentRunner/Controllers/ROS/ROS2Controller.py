import os
import sys
import time
import signal
import subprocess
from subprocess import Popen
from pathlib import Path
from ExperimentRunner.Utilities.Utils import Utils
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ROS2Controller(IROSController):
    sim_poll_proc: Popen = None

    def get_gazebo_time(self):
        if not self.sim_poll_proc:
            dir_path = os.path.dirname(os.path.realpath(__file__)) + '/../Experiment/Run'
            self.sim_poll_proc = subprocess.Popen(f"{sys.executable} {dir_path}/PollSimRunning.py", shell=True)

        # TODO: check return code if non-zero (error)
        if self.sim_poll_proc.poll() is not None:
            return 2
        else:
            return 0

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
        subprocess.call(f"ros2 lifecycle set {bag_name} shutdown", shell=True, stdout=Utils.FNULL,
                        stderr=subprocess.STDOUT)

    def ros_shutdown(self): # TODO: Graceful exit of roslaunch file not working
        output.console_log("Terminating roslaunch launch file...")
        subprocess.call("ros2 service call /reset_simulation std_srvs/srv/Empty {}", shell=True)

        self.roslaunch_proc.send_signal(signal.SIGINT)
        try:
            pid = self.roslaunch_proc.pid
            os.kill(pid, signal.SIGINT)
        except:
            output.console_log("Terminating process by PID unsuccessful")

        while self.roslaunch_proc.poll() is None:
            output.console_log_animated("Waiting for graceful exit...")

        output.console_log("Roslaunch launch file successfully terminated!")
