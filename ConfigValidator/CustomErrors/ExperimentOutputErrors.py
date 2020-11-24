from ConfigValidator.CustomErrors.BaseError import BaseError
from ExperimentOrchestrator.Misc.BashHeaders import BashHeaders

class ExperimentOutputPathAlreadyExistsError(BaseError):
    def __init__(self):
        super().__init__(
            "The experiment_path (output_path + experiment_name) already exists!\n" +
            "The config file and the CSV found in the output path do not seem to correspond!\n\n" +
            "Experiment stopped to prevent " + 
            BashHeaders.UNDERLINE + "overwriting existing results" + BashHeaders.ENDC + "."
        )

class ExperimentOutputFileDoesNotExistError(BaseError):
    def __init__(self):
        super().__init__("The " + BashHeaders.UNDERLINE + "experiment_path" + BashHeaders.ENDC + BashHeaders.FAIL + 
                            " (experiment output folder) exists, but the " + 
                            BashHeaders.UNDERLINE + "run_table.csv" + BashHeaders.ENDC + BashHeaders.FAIL +
                            " does not exist.\n" +
                            "Robot-runner cannot restart!")