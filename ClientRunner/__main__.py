import os
import sys
import json
import time
import signal
import subprocess
import multiprocessing
from subprocess import Popen
from std_msgs.msg import Empty
from multiprocessing import Event
from subprocess import CalledProcessError

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../RemoteRunner/')
from Procedures.ProcessProcedure import ProcessProcedure
from Procedures.OutputProcedure import OutputProcedure as output

from Scripts.PollExperimentEnd import ros_version
from Scripts.PollExperimentEnd import poll_exp_end_ROS1
from Scripts.PollExperimentEnd import poll_exp_end_ROS2

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

# ======================
# MAIN ROBOTCLIENT CLASS
# ======================
class RobotClient:
    verbose: bool = False
    launch_command: str
    launch_file_path: str
    roscore_node_name: str

    roslaunch_proc: Popen
    exp_end_proc: Popen

    dir_path = os.path.dirname(os.path.realpath(__file__))

    def __init__(self, config_path, verbose=False):
        self.verbose = verbose
        self.data = self.load_json(config_path)
        self.launch_command = self.get_value_for_key('launch_command')
        self.launch_file_path = self.get_value_for_key('launch_file_path')

        self.do_experiment()

        output.console_log_bold("Experiment_end signal received, exiting...")
        sys.exit(0)

    # Wait until ROS MASTER (roscore) is available (REMOTE PC)
    def wait_for_roscore(self):
        while not self.is_roscore_ready():
            output.console_log_animated("Waiting for ROS Master...")

        output.console_log_bold("ROS Master ready!", empty_line=True)

    def do_experiment(self):
        while True:
            if ros_version == 1: self.wait_for_roscore()

            target = poll_exp_end_ROS1 if ros_version == 1 else poll_exp_end_ROS2

            event_end = event_continue = Event()
            worker_exp_end = multiprocessing.Process(target=target, args=[event_end,event_continue,])
            worker_exp_end.start()

            self.do_run()

            output.console_log_bold("Waiting for experiment status from REMOTE PC...\n", empty_line=True)
            worker_exp_end.join()

            if event_end.is_set():
                break

    def do_run(self):
        def get_pid(name):
            return int(subprocess.check_output(["pgrep",name]))

        cmd_roslaunch = "roslaunch" if ros_version == 1 else "ros2 launch"
        cmd_roslaunch += f" {self.launch_file_path}"
        if not self.launch_file_path: cmd_roslaunch = self.launch_command

        output.console_log_bold(f"Launching with: {cmd_roslaunch}")

        if self.verbose:
            self.roslaunch_proc = subprocess.Popen(cmd_roslaunch.split(' '), preexec_fn=os.setsid)
        else:
            self.roslaunch_proc = subprocess.Popen(cmd_roslaunch.split(' '), 
                                                    stdout=open(os.devnull, 'w'), 
                                                    stderr=subprocess.STDOUT,
                                                    preexec_fn=os.setsid)
        
        run_completed_proc = subprocess.Popen(f"{sys.executable} {self.dir_path}/Scripts/PollRunCompletion.py", shell=True)

        # Wait for run_completed signal from REMOTE PC
        while run_completed_proc.poll() is None:
            output.console_log_animated("Waiting for run to complete...")

        os.killpg(os.getpgid(get_pid("ros2")), signal.SIGINT)
        # self.roslaunch_proc.send_signal(signal.SIGINT)
        # self.roslaunch_proc.wait()
        #ProcessProcedure.process_kill_by_name("ros2")

        while self.roslaunch_proc.poll() is None:
            output.console_log_animated(f"Waiting for '{cmd_roslaunch}' to exit...")

        output.console_log_bold("Run completed!", empty_line=True)

    # ROS1 ONLY (checks if ROS MASTER on REMOTE PC is READY)
    def is_roscore_ready(self):
        try:
            nodes_output = subprocess.check_output('rosnode list'.split(' '))
            available_nodes = str(nodes_output)
            return "/rosout" in available_nodes
        except CalledProcessError:
            return False

    # ==== UTILS ====
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
    elif argv_count == 2: RobotClient(sys.argv[1])                                              # Correct usage, no verbose
    elif argv_count == 3 and sys.argv[1] == '--verbose': RobotClient(sys.argv[2], verbose=True) # Correct usage, verbose
    else:  # Incorrect usage, display error and exit
        output.console_log("Incorrect usage, please run with --help to view possible arguments")
        sys.exit(0)