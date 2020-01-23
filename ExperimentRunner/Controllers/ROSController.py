import os
import rospy
import signal
import subprocess
from pathlib import Path
from std_msgs.msg import Bool
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ROSController:
    #def __init__(self, launch_file: Path): # add log_file

    def start_fresh_run(self, launch_file: Path): # add log_file to output roslaunch pipe to
        try:
            # TODO: Make sure /tmp/robot_runner exists
            FNULL = open(os.devnull, 'w')
            self.process = subprocess.Popen(
                                f"roslaunch --pid=/tmp/robot_runner/roslaunch.pid {launch_file}",
                                shell=True, stdout=FNULL, stderr=subprocess.STDOUT
                            )

            # TODO: Wait for ROS Master to be up and running
            # TODO: In case of sim: wait for Gazebo
            self.listen_for_run_completion()
        except Exception as e:
            output.console_log(e)

    def listen_for_run_completion(self):
        try:
            rospy.init_node("robot_runner")
            self.run_sub = rospy.Subscriber("/robot_runner/run_completed", Bool, self.run_completed)

            # rospy.on_shutdown(self.ros_shutdown())
            while not rospy.is_shutdown():
                output.console_log_loading_animated("Waiting for run to complete...")

            self.ros_shutdown()
        except:
            output.console_log("Could not initialise ROS node...")

    def run_completed(self, completed: Bool):
        if completed:
            output.console_log("Run completed")
            rospy.signal_shutdown("Run completed")

    def ros_shutdown(self):
        # TODO: Change from 'terminated' to: 'terminating...' and wait for
        # TODO: actual confirmation everything has closed down gracefully
        self.run_sub.unregister()
        output.console_log("ROS Node terminated...")
        self.process.send_signal(signal.SIGINT)
        with open('/tmp/robot_runner/roslaunch.pid', 'r') as myfile:
            pid = int(myfile.read())
            os.kill(pid, signal.SIGINT)

        output.console_log("Experiment run instance (roslaunch) terminated")
