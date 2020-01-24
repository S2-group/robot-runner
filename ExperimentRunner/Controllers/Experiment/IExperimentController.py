import time
from std_msgs.msg import Bool
from abc import ABC, abstractmethod
from ExperimentRunner.Models.ExperimentConfig import ExperimentConfig
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Controllers.ROS.ROS1Controller import ROS1Controller
from ExperimentRunner.Controllers.ROS.ROS2Controller import ROS2Controller
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class IExperimentController(ABC):
    config: ExperimentConfig = None
    ros: IROSController = None
    run_completed_topic: str = "/robot_runner/run_completed"
    running: bool = False
    timed_stop: bool = False

    def __init__(self, config: ExperimentConfig):
        self.config = config
        self.ros = ROS1Controller() if config.ros_version == 1 else ROS2Controller()
        super(IExperimentController, self).__init__()

    @abstractmethod
    def do_experiment(self):
        pass

    @abstractmethod
    def run_completed(self, data: Bool):
        pass

    def run_start(self):
        self.running = True

    def run_stop(self):
        self.running = False

    def run_wait_completed(self):
        start_time = time.time() * 1000
        while self.running:
            output.console_log_animated("Waiting for run to complete...")
            if self.timed_stop:
                diff = (time.time() * 1000) - start_time
                self.running = (diff <= self.config.duration)

        output.console_log("Run completed!", empty_line=True)
