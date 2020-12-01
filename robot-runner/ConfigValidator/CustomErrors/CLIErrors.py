from ConfigValidator.CustomErrors.BaseError import BaseError

class CommandNotRecognisedError(BaseError):
    def __init__(self):
        super().__init__("The command entered by the user is not recognised")

class InvalidUserSpecifiedPathError(BaseError):
    def __init__(self, path):
        super().__init__("The user specified path is invalid or the user does not have the correct permissions" +
                        f"\n{path}")

class InvalidConfigTypeSpecifiedError(BaseError):
    def __init__(self):
        super().__init__("The specified config type did not match: robot OR base")