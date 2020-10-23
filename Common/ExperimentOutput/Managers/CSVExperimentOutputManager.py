import csv
from typing import List

from Common.ExperimentOutput.Models.ExperimentModel import ExperimentModel
from Common.ExperimentOutput.Managers.BaseExperimentOutputManager import BaseExperimentOutputManager

class CSVExperimentOutputManager(BaseExperimentOutputManager):
    def create_experiment_run_table(self, experiment_model: ExperimentModel):
        run_table = experiment_model.get_experiment_run_table()
        with open('test.csv', 'w', newline='') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            wr.writerows(run_table)

        return run_table

    # TODO: Nice To have
    def shuffle_experiment_run_table(self):
        pass

    def append_row_to_experiment_run_table(self):
        pass

    