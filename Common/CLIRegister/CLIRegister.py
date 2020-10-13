import inspect
from enum import Enum
from typing import List
from shutil import copyfile
from tabulate import tabulate
from abc import ABC, abstractmethod

from Common.Config.BasestationConfig import BasestationConfig
from Common.Config.RobotConfig import RobotConfig

from Common.Misc.BashHeaders import BashHeaders
from Common.Misc.PathValidation import is_path_exists_or_creatable_portable
from Common.Procedures.OutputProcedure import OutputProcedure as output

from Common.CustomErrors.CLIErrors import *

class ConfigCreate:
    @staticmethod
    def description_params() -> str:
        return "<robot OR base> [path_to_user_specified_dir]"

    @staticmethod
    def description_short() -> str:
        return "Creates a config file in the current, or user-specified, directory"

    @staticmethod
    def description_long() -> str:
        output.console_log_bold("... TODO give usage instructons ...")

    @staticmethod
    def execute(args=None) -> None:
        try:
            config_type = args[2]
            destination = ""
            if len(args) == 3:
                destination = "./"
            elif len(args) == 4:
                destination = args[3]
            else:
                raise CommandNotRecognisedError
        except:
            raise CommandNotRecognisedError
            
        if config_type != "robot" and \
            config_type != "base" and \
            config_type != "basestation":
            raise InvalidConfigTypeSpecifiedError()

        if destination[-1] != "/":
            destination += "/"

        if not is_path_exists_or_creatable_portable(destination):
            raise InvalidUserSpecifiedPathError(destination)
        
        module = BasestationConfig if config_type == "base" or config_type == "basestation" else RobotConfig
        src = inspect.getmodule(module).__file__
        destination += src.split('/')[-1]
        copyfile(src, destination)

class Prepare:
    @staticmethod
    def description_params() -> str:
        return ""

    @staticmethod
    def description_short() -> str:
        return "Prepare system -- Install all dependencies"

    @staticmethod
    def description_long() -> str:
        output.console_log_bold("Prepare will install all the user's required dependencies to be able to run robot-runner on their system.")

    @staticmethod
    def execute(args=None) -> None:
        pass

class Help:
    @staticmethod
    def description_params() -> str:
        return ""

    @staticmethod
    def description_short() -> str:
        return "An overview of all available commands"

    @staticmethod
    def description_long() -> str:
        print(BashHeaders.BOLD + "--- ROBOT_RUNNER HELP ---" + BashHeaders.ENDC)
        print("\n%-*s  %s" % (10, "Usage:", "python robot-runner/ <path_to_config.py>"))
        print("%-*s  %s" % (10, "Utility:", "python robot-runner/ <command>"))

        print("\nAvailable commands:\n")
        print(tabulate([(k, v.description_params()) for k, v in CLIRegister.register.items()], ["Command", "Parameters"]))

        print("\nHelp can be called for each command:")
        print(BashHeaders.WARNING + "example: " + BashHeaders.ENDC + "python robot-runner/ prepare help")

    @staticmethod
    def execute(args=None) -> None:
        Help.description_long()

class CLIRegister:
    register = {
        "config-create":    ConfigCreate,
        "prepare":          Prepare,
        "help":             Help
    }

    @staticmethod 
    def parse_command(args: List):
        try:
            command_class = CLIRegister.register.get(args[1])
        except:
            raise CommandNotRecognisedError

        if len(args) == 2:
            command_class.execute()
        elif len(args) > 2:
            if args[2] == 'help':
                command_class.description_long()
            else:
                command_class.execute(args)