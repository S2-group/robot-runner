import os
import time
import signal
from pathlib import Path
from ExperimentRunner.Procedures.ProcessProcedure import ProcessProcedure
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Controllers.Output.OutputController import OutputController as output


class ROS2Controller(IROSController):
    def roslaunch_launch_file(self, launch_file: Path):
        output.console_log(f"ros2 launch {launch_file}")
        command = f"ros2 launch {launch_file}"
        self.roslaunch_proc = ProcessProcedure.subprocess_spawn(command, "ros2_launch_file")

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        file_path += "-ros2"
        output.console_log(f"Rosbag2 starts recording...")
        output.console_log_bold("Recording topics: ")
        # Build 'rosbag record -O filename [/topic1 /topic2 ...] __name:=bag_name' command
        command = f" ros2 bag record --output {file_path}"
        for topic in topics:
            command += f" {topic}"
            output.console_log_bold(f" * {topic}")
        command += f" __name:={bag_name}"

        ProcessProcedure.subprocess_spawn(command, "ros2bag_record")
        time.sleep(1)  # Give rosbag recording some time to initiate

    def rosbag_stop_recording_topics(self, bag_name):
        output.console_log(f"Stop recording rosbag on ROS node: {bag_name}")
        ProcessProcedure.subprocess_call(f"ros2 lifecycle set {bag_name} shutdown", "ros2bag_kill")

    def ros_shutdown(self): # TODO: Graceful exit of roslaunch file not working
        output.console_log("Terminating roslaunch launch file...")
        ProcessProcedure.subprocess_call("ros2 service call /reset_simulation std_srvs/srv/Empty {}", "ros2_reset_call")

        ProcessProcedure.process_kill_by_name("gzserver")
        ProcessProcedure.process_kill_by_name("gzclient")
        ProcessProcedure.process_kill_by_name("_ros2_daemon")
        ProcessProcedure.process_kill_by_name("ros2")
        ProcessProcedure.process_kill_by_cmdline("/opt/ros/")

        while ProcessProcedure.process_is_running("gzserver") or \
                ProcessProcedure.process_is_running("gzclient") or \
                ProcessProcedure.process_is_running("ros2") or \
                ProcessProcedure.process_is_running("_ros2_daemon"):
            output.console_log_animated("Waiting for graceful exit...")

        output.console_log("Roslaunch launch file successfully terminated!")
