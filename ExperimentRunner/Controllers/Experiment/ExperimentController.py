import time
from ExperimentRunner.Models.ConfigModel import ConfigModel
from ExperimentRunner.Controllers.Experiment.Run.SimRunController import SimRunContoller
from ExperimentRunner.Controllers.Output.OutputController import OutputController as output
from ExperimentRunner.Controllers.Experiment.Run.NativeRunController import NativeRunController


class ExperimentController:
    config: ConfigModel = None

    def __init__(self, config: ConfigModel):
        self.config = config

    # ===== Experiment =====
    def do_experiment(self):
        self.config.exp_dir.mkdir(parents=True, exist_ok=True)

        current_run: int = 1
        while current_run <= self.config.replications:
            run_controller = SimRunContoller(self.config, current_run) \
                if self.config.use_simulator else NativeRunController(self.config, current_run)

            run_controller.do_run()

            current_run += 1

            time_btwn_runs = self.config.time_between_run
            if time_btwn_runs > 0 and time_btwn_runs is not None:
                output.console_log_bold(f"Run fully ended, waiting for: {time_btwn_runs}ms == {time_btwn_runs / 1000}s")
                time.sleep(time_btwn_runs / 1000)