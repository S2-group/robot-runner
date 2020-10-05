from Common.Misc.BashHeaders import BashHeaders

class BaseError(Exception):
    def __init__(self, message):
        super().__init__(BashHeaders.FAIL + '[FAIL]: ' + BashHeaders.ENDC + 
            'ROBOT_RUNNER ENCOUNTERED AN ERROR!\n\n' + BashHeaders.FAIL + message + BashHeaders.ENDC)

class CommandNotRecognisedError(BaseError):
    def __init__(self):
        super().__init__("The command entered by the user is not recognised")

class ConfigInvalidClassNameError(BaseError):
    def __init__(self):
        super().__init__("The config file specified does not have a valid config class name as expected (ClientConfig / RemoteConfig)")

class ConfigInvalidError(BaseError):
    def __init__(self, attribute_in_question, found, expected):
        super().__init__("The config attribute " + 
                            BashHeaders.UNDERLINE + attribute_in_question + BashHeaders.ENDC + BashHeaders.FAIL +
                            " was found to be invalid.\n" +
                            "%-*s  %s\n" % (10, "FOUND:", found) +
                            "%-*s  %s" % (10, "EXPECTED:", expected) + BashHeaders.ENDC)