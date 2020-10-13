from Common.Misc.BashHeaders import BashHeaders
from Common.CustomErrors.BaseError import BaseError

class ExperimentOutputPathAlreadyExists(BaseError):
    def __init__(self):
        super().__init__(
            "The experiment_path (output_path + experiment_name) already exists!\n" +
            "Experiment stopped to prevent " + 
            BashHeaders.UNDERLINE + "overwriting existing results" + BashHeaders.ENDC + "."
        )