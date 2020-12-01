from ExperimentOrchestrator.Misc.BashHeaders import BashHeaders
from ConfigValidator.CustomErrors.BaseError import BaseError

class ConfigBaseError(BaseError):
    def __init__(self, text: str):
        super().__init__(text)

class ConfigInvalidError(ConfigBaseError):
    def __init__(self):
        super().__init__("Config found to be invalid, please refer to the config attribute table.")

class ConfigInvalidClassNameError(ConfigBaseError):
    def __init__(self):
        super().__init__("The config file specified does not have a valid config class name as expected (RobotRunnerConfig)")

class ConfigAttributeInvalidError(ConfigBaseError):
    def __init__(self, attribute_in_question, found, expected):
        super().__init__("INVALID config attribute " + 
                            BashHeaders.UNDERLINE + attribute_in_question + BashHeaders.ENDC + BashHeaders.FAIL + "\n" +
                            "%-*s  %s\n" % (10, "FOUND:", found) +
                            "%-*s  %s" % (10, "EXPECTED:", expected) + BashHeaders.ENDC)