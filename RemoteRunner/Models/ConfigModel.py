import os
import sys
import json
from typing import List
from pathlib import Path
from datetime import datetime
from Models.RunScriptModel import RunScriptModel
from Procedures.OutputProcedure import OutputProcedure as output


###     =========================================================
###     |                                                       |
###     |                      ConfigModel                      |
###     |       - Represent config.json file as a runtime       |
###     |         model for intellisense and more rigid control |
###     |                                                       |
###     |       - Fuctionality to read JSON file and set the    |
###     |         given model variables                         |
###     |                                                       |
###     |       * Any functionality regarding parsing JSON to   |
###     |         the runtime model should be added here        |
###     |                                                       |
###     =========================================================
class ConfigModel:
    use_simulator: bool
    name: str
    ros_version: int
    replications: int
    duration: int
    launch_file_path: Path
    run_script_model: RunScriptModel

    topics_must_be_available: List[str]
    nodes_must_be_available: List[str]

    output_path: Path
    topics_to_record: List[str]
    time_between_run: int

    exp_dir: Path

    def __init__(self, config_path):
        now = datetime.now().strftime("%d_%m_%Y-%H:%M:%S")
        self.data = self.load_json(config_path)

        # ===== LOAD DEFAULT CONFIG VALUES =====
        use_sim = self.get_value_for_key('use_simulator')
        self.use_simulator = (use_sim == "true")
        self.name = self.get_value_for_key('name')

        try:
            self.ros_version = int(os.environ['ROS_VERSION'])
        except ValueError:
            output.console_log_bold("Unknown value for $ROS_VERSION env variable")
            sys.exit(1)

        self.replications = self.get_value_for_key('replications')
        self.duration = self.get_value_for_key('duration')

        self.launch_file_path = Path(self.get_value_for_key('launch_file_path'))

        script_json = self.get_value_for_key('run_script')
        self.run_script_model = RunScriptModel(script_json['path'], script_json['args'])

        self.topics_must_be_available = self.get_value_for_key('topics_must_be_available')
        self.nodes_must_be_available = self.get_value_for_key('nodes_must_be_available')

        self.output_path = Path(self.get_value_for_key('output_path'))

        self.topics_to_record = self.get_value_for_key('topics_to_record')

        self.time_between_run = self.get_value_for_key('time_between_run')

        # Build experiment specific, unique path
        self.exp_dir = Path(str(self.output_path.absolute()) + f"/{self.name}-{now}")
        output.console_log("Experiment config successfully loaded in")
        output.console_log_tabulate(self.data)

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
            output.console_log(f"Value requested from config.json with unknown key: ['{key}']")
            sys.exit(0)
