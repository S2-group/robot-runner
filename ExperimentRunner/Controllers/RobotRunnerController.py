from pathlib import Path
from ExperimentRunner.Utilities.Utils import Utils
from ExperimentRunner.Models.ExperimentConfig import ExperimentConfig
from ExperimentRunner.Controllers.Experiment.IExperimentController import IExperimentController
from ExperimentRunner.Controllers.Experiment.SimExperimentController import SimExperimentController
from ExperimentRunner.Controllers.Experiment.NativeExperimentController import NativeExperimentController


class RobotRunnerController:
    exp_controller: IExperimentController  # Abstract ExperimentController which can be either Sim or Native at runtime

    def __init__(self, config_path: Path):
        self.set_exp_controller(ExperimentConfig(config_path.absolute()))

    def set_exp_controller(self, config: ExperimentConfig):
        self.exp_controller = SimExperimentController(config) if config.use_simulator else NativeExperimentController(config)

    def do_experiment(self):
        Utils.create_dir(self.exp_controller.config.exp_dir)
        self.exp_controller.do_experiment()
