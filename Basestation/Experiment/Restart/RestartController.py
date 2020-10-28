from Common.Config.BasestationConfig import BasestationConfig
from Common.ExperimentOutput.Managers.BaseExperimentOutputManager import BaseExperimentOutputManager
from Common.ExperimentOutput.Managers.CSVExperimentOutputManager import CSVExperimentOutputManager
from Common.Misc.DictConversion import pop_from_each_dict_in_list

class RestartController:
    @staticmethod
    def are_config_and_restart_csv_equal(config: BasestationConfig) -> bool:
        data_manager = CSVExperimentOutputManager()
        csv_run_table = data_manager.read_run_table_from_csv()
        config_run_table = config.create_run_table()

        if set(csv_run_table[-1].keys()) == set(config_run_table[-1].keys()):
            csv_run_table = pop_from_each_dict_in_list(csv_run_table, '__done')         # Obviously some runs will be done, so __done = 1
            config_run_table = pop_from_each_dict_in_list(config_run_table, '__done')   # Obviously newly generated, all runs will be __done = 0      
                                                                                        # Need to be popped, otherwise '==' operator will always return false.
            return csv_run_table == config_run_table
        else:
            return False