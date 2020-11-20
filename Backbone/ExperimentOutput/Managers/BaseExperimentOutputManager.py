from Backbone.Architecture.Singleton import SingletonABCMeta

class BaseExperimentOutputManager(metaclass=SingletonABCMeta):
    _experiment_path: str = None

    def set_experiment_output_path(self, experiment_output_path: str):
        self._experiment_path = experiment_output_path
