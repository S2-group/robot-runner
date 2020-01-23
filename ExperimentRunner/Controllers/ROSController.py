import os
import rospy
import signal
import subprocess
from pathlib import Path
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ROSController:
    #def __init__(self, launch_file: Path): # add log_file

    def start_fresh_run(self, launch_file: Path): # add log_file to output roslaunch pipe to
        try:
            FNULL = open(os.devnull, 'w')
            subprocess.call(f"roslaunch {launch_file}", shell=True, stdout=FNULL, stderr=subprocess.STDOUT)
            output.console_log("Succesfully started launch file")
            self.listen_for_run_completion()
        except Exception as e:
            output.console_log(e)

    def listen_for_run_completion(self):
        try:
            rospy.init_node("robot_runner")
            self.run_sub = rospy.Subscriber("/run_completed", self.run_completed)

            rospy.on_shutdown(self.ros_shutdown())
            rospy.spin()
        except:
            output.console_log("Could not initialise ROS node...")

    def run_completed(self):
        output.console_log("Run completed!")
        rospy.signal_shutdown("Run completed")

    def ros_shutdown(self):
        self.run_sub.unregister()
        output.console_log("ROS Node terminated...")
        # self.process.send_signal(signal.SIGINT)
        output.console_log("Experiment run instance (roslaunch) terminated")
