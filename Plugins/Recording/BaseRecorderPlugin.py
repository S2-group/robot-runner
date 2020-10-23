from Common.Architecture.Singleton import SingletonABCMeta
from abc import abstractmethod

class BaseRecorderPlugin(metaclass=SingletonABCMeta):
    @abstractmethod
    def start_recording(self) -> None:
        pass

    @abstractmethod
    def stop_recording(self):
        pass