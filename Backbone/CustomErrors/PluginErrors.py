from Backbone.CustomErrors.BaseError import BaseError

class PluginBaseError(BaseError):
    def __init__(self, text: str):
        super().__init__(text)

class PluginTopicRecorderAlreadyRecording(BaseError):
    def __init__(self):
        super().__init__("The TopicRecorderPlugin was already recording when start_recording was called.")