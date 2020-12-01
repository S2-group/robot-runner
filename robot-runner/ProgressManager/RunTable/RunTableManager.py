from ConfigValidator.Config.RobotRunnerConfig import RobotRunnerConfig
from ProgressManager.Output.CSVOutputManager import CSVOutputManager

class RunTableManager:
    @staticmethod
    def are_config_and_restart_csv_equal(config: RobotRunnerConfig) -> bool:
        data_manager = CSVOutputManager()
        csv_run_table = data_manager.read_run_table_from_csv()
        config_run_table = config.create_run_table()

        return set(csv_run_table[-1].keys()) == set(config_run_table[-1].keys())