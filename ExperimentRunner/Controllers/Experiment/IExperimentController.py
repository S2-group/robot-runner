import time
import rospy
from abc import ABC, abstractmethod
from ExperimentRunner.Models.ExperimentConfig import ExperimentConfig
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Controllers.ROS.ROS1Controller import ROS1Controller
from ExperimentRunner.Controllers.ROS.ROS2Controller import ROS2Controller
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class IExperimentController(ABC):
    config: ExperimentConfig
    ros: IROSController

    def __init__(self, config: ExperimentConfig):
        self.config = config
        self.ros = ROS1Controller() if config.ros_version == 1 else ROS2Controller()
        super(IExperimentController, self).__init__()

    @abstractmethod
    def do_experiment(self):
        pass

    @abstractmethod
    def run_completed(self):
        pass

    def run_wait_completed(self):
        while not rospy.is_shutdown():
            output.console_log_animated("Waiting for run to complete...")
        output.console_log("Run completed!")

    def wait_for_simulation(self):
        while rospy.Time.now() == rospy.Time():
            output.console_log_animated("Waiting for simulation to be running...")
            time.sleep(1)
        output.console_log("Simulation detected to be running, everything is ready for experiment!")
