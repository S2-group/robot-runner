import os
import sys
import time
import subprocess
from RemoteRunner.Models.ConfigModel import ConfigModel
from RemoteRunner.Controllers.Experiment.Run.SimRunController import SimRunContoller
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output
from RemoteRunner.Controllers.Experiment.Run.NativeRunController import NativeRunController


class ExperimentController:
    config: ConfigModel = None

    def __init__(self, config: ConfigModel):
        self.config = config

    # ===== Experiment =====
    def do_experiment(self):
        self.config.exp_dir.mkdir(parents=True, exist_ok=True)

        current_run: int = 1
        while current_run <= self.config.replications:
            run_controller = None
            if self.config.use_simulator:
                run_controller = SimRunContoller(self.config, current_run)
            else:
                run_controller = NativeRunController(self.config, current_run)

            run_controller.do_run()

            current_run += 1

            time_btwn_runs = self.config.time_between_run
            if time_btwn_runs > 0 and time_btwn_runs is not None:
                output.console_log_bold(f"Run fully ended, waiting for: {time_btwn_runs}ms == {time_btwn_runs / 1000}s")
                time.sleep(time_btwn_runs / 1000)

        self.signal_experiment_end()

    def signal_experiment_end(self):
        output.console_log("Experiment ended, signalling end to robot...")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        exp_end_proc = subprocess.Popen(f"{sys.executable} {dir_path}/Scripts/SignalExperimentEnd.py", shell=True)

        while exp_end_proc.poll() is not None:
            output.console_log_animated("Waiting for robot to confirm experiment end...")

        output.console_log_bold("Successfully ended experiment!")
