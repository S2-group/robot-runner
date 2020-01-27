import os
import sys
import time
import rospy
import signal
import subprocess
from pathlib import Path
from rospy import Subscriber
from roslaunch import RLException
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ROS1Controller(IROSController):
    FNULL = open(os.devnull, 'w')  # block output from showing in terminal
    roscore_proc = None
    rosbag_proc = None
    roslaunch_proc = None
    roslaunch_pid: str = "/tmp/roslaunch.pid"
    subscribed_topics = []

    def roslaunch_launch_file(self, launch_file: Path):  # TODO: add log_file to output roslaunch pipe to
        output.console_log(f"Roslaunch {launch_file}")
        command = f"roslaunch --pid=/tmp/roslaunch.pid {launch_file}"
        try:
            self.roslaunch_proc = subprocess.Popen(command, shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)
        except RLException:
            output.console_log("Something went wrong launching the launch file.")
            sys.exit(1)

    def roscore_start(self):
        output.console_log("Starting roscore (ROS Master)...")
        self.roscore_proc = subprocess.Popen('roscore', shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)
        time.sleep(1)  # wait to be sure roscore_start is really launched

    def rosinit_robot_runner_node(self):
        try:
            rospy.init_node("robot_runner")
            output.console_log("Successfully initialised ROS node: robot_runner")
        except:
            output.console_log("Could not initialise robot_runner ROS node...")

    def subscribe_to_topic(self, topic, datatype, callback):
        output.console_log(f"Subscribed to topic: {topic}")
        sub = rospy.Subscriber(topic, datatype, callback)
        self.subscribed_topics.append(sub)
        return sub

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):  # rosbag record /topic1 /topic2
        output.console_log(f"Rosbag starts recording...")  # TODO: output what topics are recorded
        # Build 'rosbag record -O filename [/topic1 /topic2 ...] __name:=bag_name' command
        command = f"rosbag record -O {file_path}"
        for topic in topics:
            command += f" {topic}"
        command += f" __name:={bag_name}"

        try:
            self.rosbag_proc = subprocess.Popen(command, shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)
            time.sleep(1)  # Give rosbag recording some time to initiate
        except RLException:
            output.console_log("Something went wrong recording topics to rosbag")
            sys.exit(1)

    def rosbag_stop_recording_topics(self, bag_name):
        output.console_log(f"Stop recording rosbag on ROS node: {bag_name}")
        subprocess.call(f"rosnode kill {bag_name}", shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)

    def ros_shutdown(self):
        output.console_log("Complete shutdown of all run-related ROS instances initiated...")
        output.console_log("Unregistering all subscribed topics...")
        sub: Subscriber
        for sub in self.subscribed_topics:
            sub.unregister()

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

    def terminate_proc(self, proc: subprocess.Popen, name):
        output.console_log(f"Terminating process: {name}...")
        proc.send_signal(signal.SIGINT)
        os.kill(proc.pid, signal.SIGINT)
        while proc.poll() is None:
            output.console_log_animated(f"Waiting for graceful exit of process: {name}...")
