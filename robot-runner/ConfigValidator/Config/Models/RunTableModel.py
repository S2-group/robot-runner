import itertools
from typing import Dict, List, Tuple
from ProgressManager.RunTable.Models.RunProgress import RunProgress
from ConfigValidator.Config.Models.FactorModel import FactorModel

class RunTableModel:
    __factors:       List[FactorModel] = None
    __experiment_run_table:          List[Dict]  = None
    __exclude_variations:            List[Dict]  = None
    __data_columns:                  List[str]   = None

    def __init__(self, factors: List[FactorModel], exclude_variations: List[Dict] = None, data_columns: List[str] = None):
        self.__factors = factors
        self.__experiment_run_table = []
        self.__exclude_variations = exclude_variations
        self.__data_columns = data_columns

    def get_factors(self) -> List[FactorModel]:
        return self.__factors

    def get_experiment_run_table(self) -> List[Dict]:
        return self.__experiment_run_table

    def create_experiment_run_table(self) -> None:
        def __filter_list(filter_list: List[Tuple]):
            if self.__exclude_variations is None:
                return filter_list

            for exclusion in self.__exclude_variations:
                filter_list = [x for x in filter_list if not exclusion <= set(x)]

            return filter_list

        list_of_lists = []
        for treatment in self.__factors:
            list_of_lists.append(treatment.get_treatments())

        combinations_list = list(itertools.product(*list_of_lists))
        filtered_list = __filter_list(combinations_list)

        column_names = ['__run_id', '__done']   # Needed for robot-runner functionality
        for factor in self.__factors:
            column_names.append(factor.get_factor_name())

        if self.__data_columns:
            for data_column in self.__data_columns:
                column_names.append(data_column)

        for i in range(0, len(filtered_list)):
            row_list = list(filtered_list[i])
            row_list.insert(0, f'run_{i}')         # __run_id
            row_list.insert(1, RunProgress.TODO)   # __done

            if self.__data_columns:
                for data_column in self.__data_columns:
                    row_list.append(" ")

            self.__experiment_run_table.append(dict(zip(column_names, row_list)))
