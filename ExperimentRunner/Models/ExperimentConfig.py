import sys
import json
from pathlib import Path
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ExperimentConfig:
    def __init__(self, config_path):
        self.load_json(config_path)

        # ===== LOAD DEFAULT CONFIG VALUES =====
        self.sim = True if self.get_value_for_key('type') == 'sim' else False
        self.replications = self.get_value_for_key('replications')
        self.duration = self.get_value_for_key('duration')
        self.launch_file_path = Path(self.get_value_for_key('launch_file_path'))
        self.output_path = Path(self.get_value_for_key('output_path'))
        self.profilers = self.get_value_for_key('profilers')
        self.scripts = self.get_value_for_key('scripts')
        self.time_between_run = self.get_value_for_key('time_between_run')

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

