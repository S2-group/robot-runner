from pathlib import Path
from RemoteRunner.Models.ConfigModel import ConfigModel
from RemoteRunner.Controllers.Experiment.ExperimentController import ExperimentController


###     =========================================================
###     |                                                       |
###     |                RobotRunnerController                  |
###     |       - Init ExperimentController with                |
###     |         correct configmodel loaded from path          |
###     |                                                       |
###     |       - Public function available for parent          |
###     |         to start experiment                           |
###     |                                                       |
###     |       * Add non-experiment related overhead           |
###     |         here, like validating config variables        |
###     |                                                       |
###     =========================================================
class RobotRunnerController:
    exp_controller: ExperimentController

    def __init__(self, config_path: Path):
        self.exp_controller = ExperimentController(ConfigModel(config_path))

    def do_experiment(self):
        self.exp_controller.do_experiment()
