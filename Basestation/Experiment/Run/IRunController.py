from Common.ExperimentOutput.Managers.CSVExperimentOutputManager import CSVExperimentOutputManager
import os
import sys
import time
import subprocess
from pathlib import Path
from subprocess import Popen
from abc import ABC, abstractmethod

from Common.Procedures.ProcessProcedure import ProcessProcedure
from Common.Procedures.OutputProcedure import OutputProcedure as output
from Common.Config.BasestationConfig import (BasestationConfig, RobotRunnerContext)


###     =========================================================
###     |                                                       |
###     |                     IRunController                    |
###     |       - Init a run based on ConfigModel and current   |
###     |         run index                                     |
###     |       - Create necessary directories (run dir)        |
###     |       - Init correct ROS controller (ROS1 or ROS2)    |
###     |         based on the ros_verison in the ConfigModel   |
###     |                                                       |
###     |       - Provide abstract, implementation specific     |
###     |         methods for Native or Sim runs to implement   |
###     |                                                       |
###     |       - Provide default, generic functions for both   |
###     |         run types (Native or Sim) like                |
###     |         run_wait_completed()                          |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific (Native or Sim) should be declared   |
###     |         here as an abstract function                  |
###     |                                                       |
###     |       * Any generic functionality between the two     |
###     |         run types should be declared here             |
###     |         as a function                                 |
###     |                                                       |
###     =========================================================
class IRunController(ABC):
    run_dir: Path = None
    current_run: int = None
    variation: tuple = None
    config: BasestationConfig = None
    run_context: RobotRunnerContext = None

    data_manager: CSVExperimentOutputManager = None

    # Needed at runtime
    running: bool = False
    timed_stop: bool = False

    # Allows for graceful exit after run_completion
    script_proc: Popen = None
    run_poll_proc: Popen = None
    block_out = open(os.devnull, 'w')  # block output from showing in terminal

    def __init__(self, variation: tuple, config: BasestationConfig, current_run: int, total_runs: int):
        self.run_dir = Path(str(config.experiment_path.absolute()) + f"/{variation['__run_id']}")
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.variation = variation
        self.config = config
        self.current_run = current_run
        self.run_context = RobotRunnerContext(self.variation, self.current_run, self.run_dir)
        self.data_manager = CSVExperimentOutputManager(str(self.config.experiment_path.absolute()))

        print(f"\n-----------------NEW RUN [{current_run} / {total_runs}]-----------------\n")

    @abstractmethod
    def do_run(self):
        pass
