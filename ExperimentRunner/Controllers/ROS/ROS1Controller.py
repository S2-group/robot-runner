import os
import time
import signal
import subprocess
from pathlib import Path
from ExperimentRunner.Procedures.ProcessProcedure import ProcessProcedure
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Controllers.Output.OutputController import OutputController as output


class ROS1Controller(IROSController):
    def roslaunch_launch_file(self, launch_file: Path):
        output.console_log(f"Roslaunch {launch_file}")
        command = f"roslaunch --pid=/tmp/roslaunch.pid {launch_file}"
        self.roslaunch_proc = ProcessProcedure.subprocess_spawn(command, "ros1_launch_file")

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        file_path += "-ros1"
        output.console_log(f"Rosbag starts recording...")
        output.console_log_bold("Recording topics: ")
        # Build 'rosbag record -O filename [/topic1 /topic2 ...] __name:=bag_name' command
        command = f"rosbag record -O {file_path}"
        for topic in topics:
            command += f" {topic}"
            output.console_log_bold(f" * {topic}")
        command += f" __name:={bag_name}"

        ProcessProcedure.subprocess_spawn(command, "ros1bag_record")
        time.sleep(1)  # Give rosbag recording some time to initiate

    def rosbag_stop_recording_topics(self, bag_name):
        output.console_log(f"Stop recording rosbag on ROS node: {bag_name}")
        ProcessProcedure.subprocess_call(f"rosnode kill {bag_name}", "ros1bag_kill")

    def ros_shutdown(self):
        output.console_log("Terminating roslaunch launch file...")
        subprocess.call("rosservice call /gazebo/reset_simulation \"{}\"", shell=True)
        self.roslaunch_proc.send_signal(signal.SIGINT)

        try:
            with open("/tmp/roslaunch.pid", 'r') as myfile:
                pid = int(myfile.read())
                os.kill(pid, signal.SIGINT)
        except FileNotFoundError:
            output.console_log("roslaunch pid not found, continuing normally...")

        while self.roslaunch_proc.poll() is None:
            output.console_log_animated("Waiting for graceful exit...")

        output.console_log("Roslaunch launch file successfully terminated!")
