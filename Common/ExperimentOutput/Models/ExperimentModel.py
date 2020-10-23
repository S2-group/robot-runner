import itertools
from typing import Dict, List, Tuple
from Common.ExperimentOutput.Models.ExperimentTreatmentModel import ExperimentTreatmentModel


class ExperimentModel:
    __treatments: List[ExperimentTreatmentModel] = None
    __experiment_run_table:          List[Tuple] = None
    __number_of_runs_per_variation:     int      = None
    __exclude_variations:            List[Dict]  = None

    def __init__(self, treatments: List[ExperimentTreatmentModel], number_of_runs_per_variation: int, exclude_variations: List[Dict] = None):
        self.__treatments = treatments
        self.__number_of_runs_per_variation = number_of_runs_per_variation
        self.__exclude_variations = exclude_variations

    def get_treatments(self) -> List[ExperimentTreatmentModel]:
        return self.__treatments

    def get_number_of_runs_per_variation(self) -> int:
        return self.__number_of_runs_per_variation

    def get_experiment_run_table(self):
        def __filter_list(filter_list: List[Tuple]):
            if self.__exclude_variations is None:
                return

            for exclusion in self.__exclude_variations:
                print(f"debug exclusion: {exclusion}")
                filter_list = [x for x in filter_list if not exclusion <= set(x)]

            self.__experiment_run_table = filter_list

        if self.__experiment_run_table is None:
            list_of_lists = []
            for treatment in self.__treatments:
                list_of_lists.append(treatment.get_column_values())

            list_of_lists.append(range(1, self.__number_of_runs_per_variation + 1))
            self.__experiment_run_table = list(itertools.product(*list_of_lists))
            __filter_list(self.__experiment_run_table)
    
        return self.__experiment_run_table
