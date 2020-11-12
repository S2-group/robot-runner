from Backbone.Misc.BashHeaders import BashHeaders
from Backbone.CustomErrors.BaseError import BaseError

class ExperimentOutputPathAlreadyExists(BaseError):
    def __init__(self):
        super().__init__(
            "The experiment_path (output_path + experiment_name) already exists!\n" +
            "The config file and the CSV found in the output path do not seem to correspond!\n\n" +
            "Experiment stopped to prevent " + 
            BashHeaders.UNDERLINE + "overwriting existing results" + BashHeaders.ENDC + "."
        )