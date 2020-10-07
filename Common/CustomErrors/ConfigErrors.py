from Common.Misc.BashHeaders import BashHeaders
from Common.CustomErrors.BaseError import BaseError

class ConfigInvalidClassNameError(BaseError):
    def __init__(self):
        super().__init__("The config file specified does not have a valid config class name as expected (ClientConfig / RemoteConfig)")

class ConfigInvalidError(BaseError):
    def __init__(self, attribute_in_question, found, expected):
        super().__init__("INVALID config attribute " + 
                            BashHeaders.UNDERLINE + attribute_in_question + BashHeaders.ENDC + BashHeaders.FAIL + "\n" +
                            "%-*s  %s\n" % (10, "FOUND:", found) +
                            "%-*s  %s" % (10, "EXPECTED:", expected) + BashHeaders.ENDC)