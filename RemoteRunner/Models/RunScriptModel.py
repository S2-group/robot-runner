import sys
from pathlib import Path
from RemoteRunner.Procedures.ProcessProcedure import ProcessProcedure
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output


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
        ProcessProcedure.subprocess_spawn(self.command, "run_script_model")
