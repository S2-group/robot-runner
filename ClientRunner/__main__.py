import os
import sys
import json
import time
import subprocess
from subprocess import Popen
from subprocess import CalledProcessError

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../RemoteRunner/')
from Procedures.OutputProcedure import OutputProcedure as output

ros_version = int(os.environ['ROS_VERSION'])


###     =========================================================
###     |                                                       |
###     |           Main (RobotClient | ClientRunner)           |
###     |       - Ensuring correct usage                        |
###     |       - SIGINT handling (graceful exit procedures)    |
###     |       - Parsing config.json to usable variables       |
###     |       - Start the launch file (.launch or .launch.py) |
###     |       - Run custom command (roslaunch or ros2 launch) |
###     |       - Correct usage of ROS1 or ROS2 is guaranteed   |
###     |         by the use of the environment variable        |
###     |       - Ensure a correct experiment end by            |
###     |         communicating with robot-runner (RemoteRunner)|
###     |         using a spawned subprocess running the node   |
###     |                                                       |
###     |       * Implementation is kept very lightweight on    |
###     |         purpose to have a minimal impact on energy    |
###     |         consumption, minimizing experiment            |
###     |         contamination                                 |
###     |                                                       |
###     =========================================================
class RobotClient:
    verbose: bool = False
    launch_command: str
    launch_file_path: str
    roscore_node_name: str

    roslaunch_proc: Popen
    exp_end_proc: Popen

    def __init__(self, config_path, verbose=False):
        self.verbose = verbose
        self.data = self.load_json(config_path)
        self.launch_command = self.get_value_for_key('launch_command')
        self.launch_file_path = self.get_value_for_key('launch_file_path')
        dir_path = os.path.dirname(os.path.realpath(__file__))

        while True:
            while ros_version == 1 and not self.is_roscore_ready():
                output.console_log_animated("Waiting for ROS Master...")

            output.console_log_bold("ROS Master ready!", empty_line=True)

            self.exp_end_proc = subprocess.Popen(f"{sys.executable} {dir_path}/Scripts/PollExperimentEnd.py",
                                                 shell=True)
            output.console_log_bold("Grace period for poll_experiment_end")
            time.sleep(5)

            if self.exp_end_proc.poll() is None:
                self.do_run()
            else:
                sys.exit(0)

    def do_run(self):
        cmd_roslaunch = "roslaunch" if ros_version == 1 else "ros2 launch"
        cmd_roslaunch += f" {self.launch_file_path}"

        if self.launch_file_path != "":
            cmd_roslaunch = self.launch_command

        if self.verbose:
            self.roslaunch_proc = subprocess.Popen(f"{cmd_roslaunch}", shell=True,
                                                   stdout=open(os.devnull, 'w'),
                                                   stderr=subprocess.STDOUT)
        else:
            self.roslaunch_proc = subprocess.Popen(f"{cmd_roslaunch} {self.launch_file_path}", shell=True)

        while self.roslaunch_proc.poll() is None:
            output.console_log_animated("Waiting for run to complete...")

    def is_roscore_ready(self):
        try:
            if self.verbose:
                nodes_output = subprocess.check_output('rosnode list', shell=True)
            else:
                nodes_output = subprocess.check_output('rosnode list', stderr=open(os.devnull, 'w'), shell=True)

            available_nodes = str(nodes_output)
            return "/rosout" in available_nodes
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
        output.console_log_bold("optional: python3 %s --verbose [PATH_TO_CONFIG.JSON]" % __file__)
        sys.exit(0)
    elif argv_count == 2:  # Correct usage, continue to program
        RobotClient(sys.argv[1])
    elif argv_count == 3 and sys.argv[1] == '--verbose':
        RobotClient(sys.argv[2], verbose=True)
    else:  # Incorrect usage, display error and exit
        output.console_log("Incorrect usage, please run with --help to view possible arguments")
        sys.exit(0)
