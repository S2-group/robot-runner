import os
import sys
import time
import subprocess
from Common.Config.BasestationConfig import BasestationConfig

from Common.Procedures.ProcessProcedure import ProcessProcedure
from Basestation.ROS.IROSController import IROSController
from Basestation.ROS.ROS1Controller import ROS1Controller
from Basestation.ROS.ROS2Controller import ROS2Controller
from Common.Procedures.OutputProcedure import OutputProcedure as output
from Basestation.Experiment.Run.SimRunController import SimRunContoller
from Basestation.Experiment.Run.NativeRunController import NativeRunController

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
    ros: IROSController

    def __init__(self, config: BasestationConfig):
        self.config = config
        self.ros = ROS1Controller() if self.config.required_ros_version == 1 else ROS2Controller()

        # TODO: Create run_schedule table and save externally. Reference and restart from crash if occurred.

        output.console_log_bold("Experiment setup completed...")
        output.console_log("Calling before_experiment config hook")
        self.config.execute_script_before_experiment()

    def do_experiment(self):
        self.config.experiment_path.mkdir(parents=True, exist_ok=True)

        current_run: int = 1
        for current_run in range(0, self.config.number_of_runs):
            run_controller = None
            run_controller = SimRunContoller(self.config, current_run, self.ros) \
                            if self.config.use_simulator else \
                            NativeRunController(self.config, current_run, self.ros)

            run_controller.do_run()

            time_btwn_runs = self.config.time_between_runs_in_ms
            if time_btwn_runs > 0:
                output.console_log_bold(f"Run fully ended, waiting for: {time_btwn_runs}ms == {time_btwn_runs / 1000}s")
                time.sleep(time_btwn_runs / 1000)

            # TODO: ROS Services signalling start / stop / continue
        
        output.console_log("Experiment completed...")
        output.console_log("Calling after_experiment config hook")
        self.config.execute_script_after_experiment()