from Basestation.Experiment.Restart.RestartController import RestartController
import sys
import time
from typing import Dict, List

from Common.ExperimentOutput.Managers.CSVExperimentOutputManager import CSVExperimentOutputManager
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
    config: BasestationConfig                   = None
    run_table: List[Dict]                       = None
    restarted: bool                             = False
    experiment_path_as_string: str              = None
    data_manager: CSVExperimentOutputManager    = None

    def __init__(self, config: BasestationConfig):
        self.config = config
        self.experiment_path_as_string = str(self.config.experiment_path.absolute())

        self.data_manager = CSVExperimentOutputManager()
        self.data_manager.set_experiment_output_path(self.experiment_path_as_string)

        self.run_table = self.config.create_run_table()
        self.create_experiment_output_folder()
        
        if not self.restarted:
            self.data_manager.write_run_table_to_csv(self.run_table)
        
        output.console_log_WARNING("Experiment run table created...")

    def do_experiment(self):
        output.console_log_OK("Experiment setup completed...")
        
        # -- Before experiment
        output.console_log_WARNING("Calling before_experiment config hook")
        self.config.before_experiment()

        # -- Experiment
        for variation in self.run_table:
            if variation['__done'] == 1:
                continue

            RunController(variation, self.config, (self.run_table.index(variation) + 1), len(self.run_table)).do_run()  # Perform run

            time_btwn_runs = self.config.time_between_runs_in_ms
            if time_btwn_runs > 0:
                output.console_log_bold(f"Run fully ended, waiting for: {time_btwn_runs}ms == {time_btwn_runs / 1000}s")
                time.sleep(time_btwn_runs / 1000)
        
        output.console_log_OK("Experiment completed...")

        # -- After experiment
        output.console_log_WARNING("Calling after_experiment config hook")
        self.config.after_experiment()

    def create_experiment_output_folder(self):
        try:
            self.config.experiment_path.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            if RestartController.are_config_and_restart_csv_equal(self.config):
                self.run_table = self.data_manager.read_run_table_from_csv()
                self.restarted = True
            else:
                raise ExperimentOutputPathAlreadyExists
