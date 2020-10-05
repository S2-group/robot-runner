import sys
from pathlib import Path
from Procedures.ProcessProcedure import ProcessProcedure
from Procedures.OutputProcedure import OutputProcedure as output


###     =========================================================
###     |                                                       |
###     |                     RunScriptModel                    |
###     |       - Represent a script, as given as a Path,       |
###     |         as a model at runtime for intellisense and    |
###     |         gauranteed safe execution of the script       |
###     |           - As the command is built-up here and the   |
###     |             run method is defined here                |
###     |                                                       |
###     |       * Any functionality regarding parsing JSON to   |
###     |         the runtime model should be added here        |
###     |                                                       |
###     =========================================================
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
