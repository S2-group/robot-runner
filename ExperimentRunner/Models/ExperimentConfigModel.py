import sys
import json
from typing import List
from pathlib import Path
from datetime import datetime
from ExperimentRunner.Models.RunScriptModel import RunScriptModel
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ExperimentConfigModel:
    name: str
    ros_version: int
    use_simulator: bool
    replications: int
    duration: int
    launch_file_path: Path
    run_script_model: RunScriptModel
    output_path: Path
    topics: List[str]
    time_between_run: int

    exp_dir: str

    def __init__(self, config_path):
        now = datetime.now().strftime("%d_%m_%Y-%H:%M:%S")
        self.load_json(config_path)

        # ===== LOAD DEFAULT CONFIG VALUES =====
        self.name = self.get_value_for_key('name')
        self.ros_version = self.get_value_for_key('ros_version')
        self.use_simulator = bool(self.get_value_for_key('use_simulator'))

        self.replications = self.get_value_for_key('replications')
        self.duration = self.get_value_for_key('duration')

        self.launch_file_path = Path(self.get_value_for_key('launch_file_path'))

        script_json = self.get_value_for_key('run_script')
        self.run_script_model = RunScriptModel(script_json['path'], script_json['args'])

        self.output_path = Path(self.get_value_for_key('output_path'))

        self.topics = self.get_value_for_key('topics')

        self.time_between_run = self.get_value_for_key('time_between_run')

        # Build experiment specific, unique path
        self.exp_dir = str(self.output_path.absolute()) + f"/{self.name}-{now}"
        output.console_log("Experiment config successfully loaded in")
        output.console_log_tabulate(self.data)

    def load_json(self, config_path):
        try:
            with open(config_path) as config_file:
                self.data = json.load(config_file)
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
