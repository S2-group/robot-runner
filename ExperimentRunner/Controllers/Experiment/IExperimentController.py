import os
import time
import subprocess
from abc import ABC, abstractmethod
from ExperimentRunner.Utilities.Utils import Utils
from ExperimentRunner.Models.ExperimentConfigModel import ExperimentConfigModel
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Controllers.ROS.ROS1Controller import ROS1Controller
from ExperimentRunner.Controllers.ROS.ROS2Controller import ROS2Controller
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class IExperimentController(ABC):
    # Variable needed for negating subprocess output
    FNULL = Utils.FNULL

    # Set on init
    config: ExperimentConfigModel = None
    ros: IROSController = None

    # Needed at runtime
    running: bool = False
    timed_stop: bool = False

    # Allows for graceful exit after run_completion
    script_proc: subprocess.Popen = None
    run_poll_proc: subprocess.Popen = None

    def __init__(self, config: ExperimentConfigModel):
        self.config = config
        self.ros = ROS1Controller() if config.ros_version == 1 else ROS2Controller()
        super(IExperimentController, self).__init__()

    # ===== Experiment =====
    def do_experiment(self):
        current_run: int = 1
        while current_run <= self.config.replications:
            self.do_run(current_run)
            current_run += 1

    # ===== Run =====
    @abstractmethod
    def do_run(self, current_run: int):
        pass

    def run_start(self):
        self.running = True

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
                self.running = self.run_poll_proc.poll() is None

        output.console_log("Run completed!", empty_line=True)
        self.run_complete_gracefully()

    def run_stop(self):
        self.running = False

    def run_script(self):
        script = self.config.run_script_model
        command = f"python3.7 {script.path}"
        for arg in script.args:
            command += f" {arg}"

        output.console_log(f"Running script: {script.path}")
        output.console_log(f"Script command: {command}")

        self.script_proc = subprocess.Popen(command, shell=True, stdout=self.FNULL, stderr=subprocess.STDOUT)

    def run_complete_gracefully(self):
        if self.script_proc:
            Utils.terminate_proc(self.script_proc, "run_script_process")

    # Stop of run
    def timed_run_stop(self):
        output.console_log_bold(f"Running experiment run for: {self.config.duration}ms == {self.config.duration / 1000}s")
        self.timed_stop = True

    def programmatic_run_stop(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.run_poll_proc = subprocess.Popen(f"python3.7 {dir_path}/Run/PollRunCompletion.py", shell=True)
