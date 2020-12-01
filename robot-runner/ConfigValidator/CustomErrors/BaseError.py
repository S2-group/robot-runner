from ExperimentOrchestrator.Misc.BashHeaders import BashHeaders

class BaseError(Exception):
    def __init__(self, message):
        super().__init__(BashHeaders.FAIL + '[FAIL]: ' + BashHeaders.ENDC + 
            'ROBOT_RUNNER ENCOUNTERED AN ERROR!\n\n' + BashHeaders.FAIL + message + BashHeaders.ENDC)