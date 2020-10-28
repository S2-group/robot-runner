from Common.CustomErrors.BaseError import BaseError
from Common.Misc.BashHeaders import BashHeaders

class ExperimentOutputFileDoesNotExistError(BaseError):
    def __init__(self):
        super().__init__("The " + BashHeaders.UNDERLINE + "experiment_path" + BashHeaders.ENDC + BashHeaders.FAIL + 
                            " (experiment output folder) exists, but the " + 
                            BashHeaders.UNDERLINE + "run_table.csv" + BashHeaders.ENDC + BashHeaders.FAIL +
                            " does not exist.\n" +
                            "Robot-runner cannot restart!")