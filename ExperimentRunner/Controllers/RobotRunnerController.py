from pathlib import Path
from ExperimentRunner.Models.ConfigModel import ConfigModel
from ExperimentRunner.Controllers.Experiment.ExperimentController import ExperimentController


class RobotRunnerController:
    exp_controller: ExperimentController

    def __init__(self, config_path: Path):
        self.exp_controller = ExperimentController(ConfigModel(config_path))

    def do_experiment(self):
        self.exp_controller.do_experiment()
