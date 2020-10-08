import os
import time
import signal
import subprocess
from pathlib import Path

from Common.Procedures.ProcessProcedure import ProcessProcedure
from Basestation.ROS.IROSController import IROSController
from Common.Procedures.OutputProcedure import OutputProcedure as output


###     =========================================================
###     |                                                       |
###     |                     ROS1Controller                    |
###     |       - Define communications with ROS1               |
###     |           - Start roscore process                     |
###     |           - Launch a launch file (.launch)            |
###     |           - Start and stop rosbag recording of topics |
###     |           - Define graceful shutdown procedure        |
###     |             for both Native and Sim runs              |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific (ROS1) should be declared here       |
###     |                                                       |
###     =========================================================
class ROS1Controller(IROSController):
    def __init__(self):
        self.roscore_start()

    def roscore_start(self):
        output.console_log("Starting ROS Master (roscore)...")
        self.roscore_proc = ProcessProcedure.subprocess_spawn("roscore", "roscore_start")
        while not ProcessProcedure.process_is_running('roscore') and \
                not ProcessProcedure.process_is_running('rosmaster') and \
                not ProcessProcedure.process_is_running('rosout'):
            output.console_log_animated("Waiting to confirm roscore running...")

        output.console_log_bold("ROS Master (roscore) is running!")
        time.sleep(1)  # Give roscore some time to initiate

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        file_path += "-ros1"
        output.console_log(f"Rosbag starts recording...")
        output.console_log_bold(f"Rosbag recording to: {file_path}")
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
