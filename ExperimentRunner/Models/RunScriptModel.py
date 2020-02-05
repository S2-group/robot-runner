import sys
from pathlib import Path
from ExperimentRunner.Procedures.ProcessProcedure import ProcessProcedure
from ExperimentRunner.Controllers.Output.OutputController import OutputController as output


class RunScriptModel:
    path: Path
    args: []
    command: str

    def __init__(self, path: Path, args: []):
        self.path = path
        self.args = args

        self.command = f"{sys.executable} {self.path}"
        for arg in self.args:
            self.command += f" {arg}"

    def run(self):
        output.console_log(f"Running script: {self.path}")
        output.console_log(f"Script command: {self.command}")
        ProcessProcedure.subprocess_spawn(self.command)
