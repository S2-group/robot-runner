import itertools
from typing import Dict, List, Tuple
from Backbone.ExperimentOutput.Models.ExperimentFactorModel import ExperimentFactorModel

class ExperimentModel:
    __factors:       List[ExperimentFactorModel] = None
    __experiment_run_table:          List[Dict]  = None
    __exclude_variations:            List[Dict]  = None

    def __init__(self, treatments: List[ExperimentFactorModel], exclude_variations: List[Dict] = None):
        self.__factors = treatments
        self.__experiment_run_table = []
        self.__exclude_variations = exclude_variations

    def get_treatments(self) -> List[ExperimentFactorModel]:
        return self.__factors

    def get_experiment_run_table(self) -> List[Dict]:
        return self.__experiment_run_table

    def create_experiment_run_table(self) -> None:
        def __filter_list(filter_list: List[Tuple]):
            if self.__exclude_variations is None:
                return

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

        for i in range(0, len(filtered_list)):
            row_list = list(filtered_list[i])
            row_list.insert(0, f'run_{i}')   # __run_id
            row_list.insert(1, 0)            # __done
            self.__experiment_run_table.append(dict(zip(column_names, row_list)))
