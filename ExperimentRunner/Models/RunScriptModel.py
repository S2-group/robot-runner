from pathlib import Path


class RunScriptModel:
    path: Path
    args: []

    def __init__(self, path: Path, args: []):
        self.path = path
        self.args = args
