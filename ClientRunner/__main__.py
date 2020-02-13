import os
import sys
import json
import subprocess
from subprocess import Popen
from subprocess import CalledProcessError

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../')
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output

ros_version = int(os.environ['ROS_VERSION'])


class RobotClient:
    launch_command: str
    launch_file_path: str
    roscore_node_name: str

    roslaunch_proc: Popen
    poll_exp_end_proc: Popen

    def __init__(self, config_path):
        self.data = self.load_json(config_path)

        self.launch_command = self.get_value_for_key('launch_command')
        self.launch_file_path = self.get_value_for_key('launch_file_path')
        self.roscore_node_name = "/rosout"

        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.poll_exp_end_proc = subprocess.Popen(f"{sys.executable} {dir_path}/Scripts/PollExperimentEnd.py", shell=True)

        while True:
            if self.poll_exp_end_proc.poll() is not None:
                self.do_run()
            else:
                subprocess.call("rosnode kill -a", shell=True)
                break

    def do_run(self):
        while not self.is_roscore_ready():
            output.console_log_animated("Waiting for ROS Master...")

        output.console_log_bold("ROS Master ready!")

        # Launch launchfile / launch command
        if self.launch_command == "":
            self.roslaunch_proc = subprocess.Popen(f"roslaunch {self.launch_file_path}", shell=True,
                                                   stdout=open(os.devnull, 'w'),
                                                   stderr=subprocess.STDOUT)
        else:
            self.roslaunch_proc = subprocess.Popen(self.launch_command, shell=True, stdout=open(os.devnull, 'w'),
                                                   stderr=subprocess.STDOUT)

        # Wait for run to finish (/rosout is unavailable again)
        while self.roslaunch_proc.poll() is None:
            output.console_log_animated("Waiting for run to complete...")

    def is_roscore_ready(self):
        command = 'rosnode list' if ros_version == 1 else 'ros2 node list'
        try:
            available_nodes = str(
                subprocess.check_output(command, stderr=open(os.devnull, 'w'), shell=True))  # TODO: ros2 node list
            return self.roscore_node_name in available_nodes
        except CalledProcessError:
            return False

    def load_json(self, config_path):
        try:
            with open(config_path) as config_file:
                return json.load(config_file)
        except FileNotFoundError:
            output.console_log("File not found, make sure file exists or path is correct...")
            sys.exit(0)
        except ValueError:
            output.console_log("Decoding JSON has failed, please check validity of config file...")
            sys.exit(0)

    def get_value_for_key(self, key):
        try:
            value = self.data[key]
            return value
        except KeyError:
            output.console_log_bold(f"Value requested from config.json with unknown key: ['{key}']")
            sys.exit(1)


if __name__ == "__main__":
    argv_count = len(sys.argv)
    if argv_count == 2 and sys.argv[1] == "--help":  # Help CLI
        output.console_log("usage: python3 %s [PATH_TO_CONFIG.JSON]" % __file__)
        sys.exit(0)
    elif argv_count == 2:  # Correct usage, continue to program
        RobotClient(sys.argv[1])
    else:  # Incorrect usage, display error and exit
        output.console_log("Incorrect usage, please run with --help to view possible arguments")
        sys.exit(0)
