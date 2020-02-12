import os
import sys
import time
import json
import subprocess
from subprocess import CalledProcessError

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '~/robot-runner/RemoteRunner')
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output


class RobotClient:
    launch_command: str
    launch_file_path: str
    roscore_node_name: str

    def __init__(self, config_path):
        self.data = self.load_json(config_path)

        self.launch_command = self.get_value_for_key('launch_command')
        self.launch_file_path = self.get_value_for_key('launch_file_path')
        self.roscore_node_name = "/rosout"

        while True:
            # check if robot-runner-finished node is available. if so: cancel.
            self.do_run()

    def do_run(self):
        while not self.is_roscore_ready():
            output.console_log_animated("Waiting for ROS Master...")
            time.sleep(0.5)

        output.console_log_bold("ROS Master ready!")

        # Launch launchfile / launch command
        if self.launch_command == "":
            subprocess.Popen(f"roslaunch {self.launch_file_path}", shell=True, stdout=open(os.devnull, 'w'),
                             stderr=subprocess.STDOUT)
        else:
            subprocess.Popen(self.launch_command, shell=True, stdout=open(os.devnull, 'w'), stderr=subprocess.STDOUT)

        # Wait for run to finish (/rosout is unavailable again)
        while self.is_roscore_ready():
            output.console_log_animated("Waiting for run to complete...")
            time.sleep(0.5)

    def is_roscore_ready(self):
        try:
            available_nodes = str(subprocess.check_output('rosnode list', shell=True))  # TODO: ros2 node list
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
