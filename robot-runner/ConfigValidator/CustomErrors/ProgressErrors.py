from ConfigValidator.CustomErrors.BaseError import BaseError

class ProgressBaseError(BaseError):
    def __init__(self, text: str):
        super().__init__(text)

class AllRunsCompletedOnRestartError(ProgressBaseError):
    def __init__(self):
        super().__init__("The experiment was restarted, but all runs are already completed.")