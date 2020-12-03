import csv
import time

from pathlib import Path
from shutil import copyfile
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ProgressManager.Output.OutputProcedure import OutputProcedure as output

class INA219Profiler:
    __path_to_data_file: str

    def __init__(self, path_to_data_file):
        self.__path_to_data_file = path_to_data_file

    def move_data_file_to_run_folder(self, context: RobotRunnerContext, file_name: str):
        copyfile(self.__path_to_data_file, str(context.run_dir.absolute()) + file_name)

    def halt_and_collect_measures_from_sd_card_and_return_data(self):
        data_file = Path(self.__path_to_data_file)
        output.console_log_WARNING(f"Waiting for SD Card to be inserted...\n\nWaiting for file to exist: {self.__path_to_data_file}")
        while not data_file.is_file():
            time.sleep(1)

        output.console_log("\n")
        output.console_log_OK("File exists!")
        
        data_array = []
        with open(self.__sd_card_path_data_file) as csvDataFile:
            csvReader = csv.reader(csvDataFile)
            for row in csvReader:
                value_array = str(row).split(',')
                data_array.append(value_array)

        return data_array