from abc import ABCMeta, abstractmethod
from pathlib import Path


class IROSController(object, metaclass=ABCMeta):
    @abstractmethod
    def roslaunch_launch_file(self, launch_file: Path):
        pass
