from Backbone.Config.RobotRunnerConfig import RobotRunnerConfig, RobotRunnerContext
from Backbone.ExperimentOutput.Managers.BaseExperimentOutputManager import BaseExperimentOutputManager
from Backbone.ExperimentOutput.Managers.CSVExperimentOutputManager import CSVExperimentOutputManager
from Backbone.Misc.DictConversion import pop_from_each_dict_in_list

class ProgressManager:
    @staticmethod
    def are_config_and_restart_csv_equal(config: RobotRunnerConfig) -> bool:
        data_manager = CSVExperimentOutputManager()
        csv_run_table = data_manager.read_run_table_from_csv()
        config_run_table = config.create_run_table()

        return set(csv_run_table[-1].keys()) == set(config_run_table[-1].keys())