import os
import sys
import time
import subprocess
from pathlib import Path
from subprocess import Popen
from abc import ABC, abstractmethod
from Procedures.ProcessProcedure import ProcessProcedure
from Controllers.ROS.IROSController import IROSController
from Controllers.ROS.ROS1Controller import ROS1Controller
from Controllers.ROS.ROS2Controller import ROS2Controller
from Procedures.OutputProcedure import OutputProcedure as output
from Models.ConfigModel import ConfigModel


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
    ros: IROSController = None
    config: ConfigModel = None

    # Needed at runtime
    running: bool = False
    timed_stop: bool = False

    # Allows for graceful exit after run_completion
    script_proc: Popen = None
    run_poll_proc: Popen = None
    block_out = open(os.devnull, 'w')  # block output from showing in terminal

    def __init__(self, config: ConfigModel, current_run: int, ros: IROSController):
        self.run_dir = Path(str(config.exp_dir.absolute()) + f"/run{current_run}")
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.current_run = current_run
        self.ros = ros
        self.config = config
        print(f"\n-----------------NEW RUN [{current_run} / {self.config.replications}]-----------------\n")

    @abstractmethod
    def do_run(self):
        pass

    def wait_for_necessary_topics_and_nodes(self):
        while not self.ros.are_nodes_available(self.config.nodes_must_be_available) and \
                not self.ros.are_topics_available(self.config.topics_must_be_available):
            output.console_log_animated("Waiting for necessary nodes and topics to be available...")

    def run_start(self):
        self.running = True

    def set_run_stop(self):
        self.timed_run_stop() if self.config.duration > 0 else self.programmatic_run_stop()

    def run_wait_completed(self):
        start_time = time.time() * 1000
        while self.running:
            output.console_log_animated("Waiting for run to complete...")

            # When run needs to stop using a timed stop
            if self.timed_stop:
                diff = (time.time() * 1000) - start_time
                self.running = (diff <= self.config.duration)

            # When run needs to stop using programmatic stop, poll if check still runs
            if self.config.duration == 0:
                self.running = self.run_poll_proc.poll() is None  # TODO: check return code non-zero (error)

        output.console_log("Run completed!", empty_line=True)
        self.run_complete_gracefully()

    def run_runscript_if_present(self):
        # Run specified run_script by user in config.json if present
        if self.config.run_script_model.path != "" and self.config.run_script_model.path is not None:
            self.config.run_script_model.run()

    def run_complete_gracefully(self):
        if self.script_proc:
            ProcessProcedure.subprocess_terminate(self.script_proc, "run_script_process")

    def timed_run_stop(self):
        output.console_log_bold(
            f"Running experiment run for: {self.config.duration}ms == {self.config.duration / 1000}s")
        self.timed_stop = True

    def programmatic_run_stop(self):
        output.console_log_bold(f"Running experiment run with programmatic stop.")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.run_poll_proc = subprocess.Popen(f"{sys.executable} {dir_path}/Scripts/PollRunCompletion.py", shell=True)
