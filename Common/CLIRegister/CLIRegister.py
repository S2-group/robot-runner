from enum import Enum
from typing import List
from abc import ABC, abstractmethod
from Common.Procedures.OutputProcedure import OutputProcedure as output
from Common.CustomErrors.CustomErrors import CommandNotRecognisedError

# Normal operation types
class CLI(Enum):
    PREPARE = 1
    HELP = 2

    @staticmethod
    def get(label: str):
        return CLI[label.upper()]

class Prepare:
    @staticmethod
    def description_short() -> str:
        return "Prepare system -- Install all dependencies"

    @staticmethod
    def description_long() -> str:
        output.console_log_bold("Prepare will install all the user's required dependencies to be able to run robot-runner on their system.")

    @staticmethod
    def execute(verbose=False) -> None:
        pass

class Help:
    @staticmethod
    def description_short() -> str:
        return "An overview of all available commands"

    @staticmethod
    def description_long() -> str:
        output.console_log_bold("--- ROBOT_RUNNER HELP ---")
        output.console_log("")
        output.console_log("%-*s  %s" % (10, "Usage:", "python robot-runner/ <path_to_config.py>"))
        output.console_log("%-*s  %s" % (10, "Utility:", "python robot-runner/ <command>"))
        output.console_log("")

        output.console_log("Available commands:")
        for cmd in CLI:
            descr = CLIRegister.register.get(cmd).description_short()
            output.console_log("* %-*s  %s" % (10, cmd.name, descr))

        output.console_log("")
        output.console_log("Help can be called for each command:")
        output.console_log("python robot-runner/ prepare help")

    @staticmethod
    def execute(verbose=False) -> None:
        Help.description_long()

class CLIRegister:
    register = {
        CLI.PREPARE:    Prepare,
        CLI.HELP:       Help
    }

    @staticmethod 
    def parse_command(args: List):
        try:
            command_class = CLIRegister.register.get(CLI.get(args[1]))
        except:
            raise CommandNotRecognisedError    

        if len(args) == 2:
            command_class.execute()
        elif len(args) > 2:
            if args[2] == 'help':
                command_class.description_long()
            else:
                raise CommandNotRecognisedError