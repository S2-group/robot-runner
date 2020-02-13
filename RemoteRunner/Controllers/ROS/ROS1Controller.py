import os
import time
import signal
import subprocess
from pathlib import Path
from RemoteRunner.Procedures.ProcessProcedure import ProcessProcedure
from RemoteRunner.Controllers.ROS.IROSController import IROSController
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output


class ROS1Controller(IROSController):
    def get_available_topics(self):
        return str(subprocess.check_output('rostopic list', shell=True))

    def get_available_nodes(self):
        return str(subprocess.check_output('rosnode list', shell=True))

    def are_nodes_available(self, node_names):
        nodes = self.get_available_nodes()
        all_available = True
        for node in node_names:
            all_available = node in nodes

        return all_available

    def are_topics_available(self, topic_names):
        topics = self.get_available_topics()
        all_available = True
        for topic in topic_names:
            all_available = topic in topics

        return all_available

    def roscore_start(self):
        output.console_log("Starting ROS Master (roscore)...")
        self.roscore_proc = ProcessProcedure.subprocess_spawn("roscore", "roscore_start")
        while not ProcessProcedure.process_is_running('roscore') and \
                not ProcessProcedure.process_is_running('rosmaster') and \
                not ProcessProcedure.process_is_running('rosout'):
            output.console_log_animated("Waiting to confirm roscore running...")

        output.console_log_bold("ROS Master (roscore) is running!")
        time.sleep(1)  # Give roscore some time to initiate

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

    def sim_shutdown(self):
        output.console_log("Shutting down sim run...")
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

        output.console_log("Sim run successfully shutdown!")

    def native_shutdown(self):
        output.console_log("Shutting down native run...")
        ProcessProcedure.subprocess_call('rosnode kill -a', "rosnode_kill")
        ProcessProcedure.process_kill_by_name('rosmaster')
        ProcessProcedure.process_kill_by_name('roscore')
        ProcessProcedure.process_kill_by_name('rosout')

        while ProcessProcedure.process_is_running('rosmaster') and \
              ProcessProcedure.process_is_running('roscore') and \
              ProcessProcedure.process_is_running('rosout'):
            output.console_log_animated("Waiting for roscore to gracefully exit...")

        output.console_log("Native run successfully shutdown!")
