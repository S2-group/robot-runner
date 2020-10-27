from Common.Architecture.Singleton import SingletonABCMeta

class BaseExperimentOutputManager(metaclass=SingletonABCMeta):
    experiment_path: str = None
    
    def __init__(self, path: str) -> None:
        self.experiment_path = path