from Common.Procedures.OutputProcedure import OutputProcedure as output
from tempfile import NamedTemporaryFile
import shutil
import csv
from typing import Dict, List

from Common.ExperimentOutput.Models.ExperimentModel import ExperimentModel
from Common.ExperimentOutput.Managers.BaseExperimentOutputManager import BaseExperimentOutputManager

class CSVExperimentOutputManager(BaseExperimentOutputManager):
    def write_run_table_to_csv(self, run_table: List[Dict]):
        with open(self.experiment_path + '/run_table.csv', 'w', newline='') as myfile:
            writer = csv.DictWriter(myfile, fieldnames=list(run_table[0].keys()))
            writer.writeheader()
            for data in run_table:
                writer.writerow(data)

    # TODO: Nice To have
    def shuffle_experiment_run_table(self):
        pass
    
    def update_row_data(self, updated_row: dict):
        tempfile = NamedTemporaryFile(mode='w', delete=False)

        with open(self.experiment_path + '/run_table.csv', 'r') as csvfile, tempfile:
            reader = csv.DictReader(csvfile, fieldnames=list(updated_row.keys()))
            writer = csv.DictWriter(tempfile, fieldnames=list(updated_row.keys()))

            for row in reader:
                if row['__run_id'] == updated_row['__run_id']:
                    writer.writerow(updated_row)
                else:
                    writer.writerow(row)

        shutil.move(tempfile.name, self.experiment_path + '/run_table.csv')
        output.console_log_WARNING(f"CSVManager: Updated row {updated_row['__run_id']}")

        # with open(self.experiment_path + '/run_table.csv', 'w', newline='') as myfile:
        #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        #     wr.writerow(updated_row)

    # for row in reader:
    # if name == row['name']:
    #     row['name'] = input("enter new name for {}".format(name))
    # # write the row either way
    # writer.writerow({'name': row['name'], 'number': row['number'], 'address': row['address']})