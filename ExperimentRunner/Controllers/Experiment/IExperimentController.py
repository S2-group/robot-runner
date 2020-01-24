from abc import ABC, abstractmethod
from ExperimentRunner.Models.ExperimentConfig import ExperimentConfig
from ExperimentRunner.Controllers.ROS.IROSController import IROSController
from ExperimentRunner.Controllers.ROS.ROS1Controller import ROS1Controller
from ExperimentRunner.Controllers.ROS.ROS2Controller import ROS2Controller


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
