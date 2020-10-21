import sys
import traceback
from typing import List
import importlib.machinery

from Common.CustomErrors.BaseError import BaseError
from Common.CLIRegister.CLIRegister import CLIRegister
from Common.Config.Validation.ConfigValidator import ConfigValidator
from Common.CustomErrors.ConfigErrors import ConfigInvalidClassNameError

from Robot.RobotController import RobotController
from Basestation.BasestationController import BasestationController

def is_no_argument_given(args: List[str]): return (len(args) == 1)
def is_config_file_given(args: List[str]): return (args[1][-3:] == '.py')
def load_and_get_config_file_as_module(args: List[str]):
    module_name = args[1].split('/')[-1].replace('.py', '')
    spec = importlib.util.spec_from_file_location(module_name, args[1]) 
    config_file = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = config_file
    spec.loader.exec_module(config_file)
    return config_file

if __name__ == "__main__":
    try: 
        if is_no_argument_given(sys.argv):
            sys.argv.append('help')
            CLIRegister.parse_command(sys.argv)
        elif is_config_file_given(sys.argv):                                # If the first arugments ends with .py -> a config file is entered
            config_file = load_and_get_config_file_as_module(sys.argv)

            if hasattr(config_file, 'BasestationConfig'):
                base_config = config_file.BasestationConfig()               # Instantiate config from injected file
                ConfigValidator.validate_base_config(base_config)           # Validate config as a BasestationConfig
                BasestationController(base_config).do_experiment()          # Instantiate controller with config and start experiment
            elif hasattr(config_file, 'RobotConfig'):
                robot_config = config_file.RobotConfig()
                ConfigValidator.validate_robot_config(robot_config)
                RobotController(robot_config)
            else:
                raise ConfigInvalidClassNameError
        else:                                                               # Else, a utility command is entered
            CLIRegister.parse_command(sys.argv)
    except BaseError as e:                                                  # All custom errors are displayed in custom format
        print(f"\n{e}")
    except:                                                                 # All non-covered errors are displayed normally
        traceback.print_exc()