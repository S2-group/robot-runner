from Common.Procedures.OutputProcedure import OutputProcedure as output
from Basestation.Experiment.ExperimentController import ExperimentController
from Common.Config.BasestationConfig import BasestationConfig

###     =========================================================
###     |                                                       |
###     |                   BaseStation                         |
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
class BasestationController:
    exp_controller: ExperimentController

    def __init__(self, basestation_config: BasestationConfig):
        self.exp_controller = ExperimentController(basestation_config)

    def do_experiment(self):
        self.exp_controller.do_experiment()
