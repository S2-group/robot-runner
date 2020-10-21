import time

from Basestation.Experiment.Run.RunController import RunController

from Common.Config.BasestationConfig import BasestationConfig
from Common.Procedures.OutputProcedure import OutputProcedure as output
from Common.CustomErrors.ExperimentErrors import ExperimentOutputPathAlreadyExists


###     =========================================================
###     |                                                       |
###     |                  ExperimentController                 |
###     |       - Init and perform runs of correct type         |
###     |       - Perform experiment overhead                   |
###     |       - Perform run overhead (time_btwn_runs)         |
###     |       - Signal experiment end to robot (ClientRunner) |
###     |                                                       |
###     |       * Experiment config that should be used         |
###     |         throughout the program is declared here       |
###     |         and should not be redeclared (only passed)    |
###     |                                                       |
###     =========================================================
class ExperimentController:
    config: BasestationConfig = None

    def __init__(self, config: BasestationConfig):
        self.config = config

        # TODO: Create run_schedule table and save externally. Reference and restart from crash if occurred.
        output.console_log_WARNING("Experiment run table created...")

    def do_experiment(self):
        try:
            self.config.experiment_path.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            raise ExperimentOutputPathAlreadyExists
        
        output.console_log_OK("Experiment setup completed...")
        
        # -- Before experiment
        output.console_log_WARNING("Calling before_experiment config hook")
        self.config.execute_script_before_experiment()

        # -- Experiment

        current_run: int = 1
        for current_run in range(1, self.config.number_of_runs + 1):
            RunController(self.config, current_run, self.ros).do_run()  # Perform run

            time_btwn_runs = self.config.time_between_runs_in_ms
            if time_btwn_runs > 0:
                output.console_log_bold(f"Run fully ended, waiting for: {time_btwn_runs}ms == {time_btwn_runs / 1000}s")
                time.sleep(time_btwn_runs / 1000)

            # TODO: ROS Services signalling start / stop / continue (native)
        
        output.console_log_OK("Experiment completed...")

        # -- After experiment
        output.console_log_WARNING("Calling after_experiment config hook")
        self.config.execute_script_after_experiment()