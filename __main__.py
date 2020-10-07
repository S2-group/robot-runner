import sys
import traceback
import importlib.machinery
from Common.CLIRegister.CLIRegister import CLIRegister

from Common.CustomErrors.CLIErrors import CommandNotRecognisedError
from Common.CustomErrors.ConfigErrors import ConfigInvalidClassNameError

from Common.Procedures.OutputProcedure import OutputProcedure as output

from Common.Config.Validation.ConfigValidator import ConfigValidator
from Common.Config.BasestationConfig import BasestationConfig
from Common.Config.RobotConfig import RobotConfig

from Basestation.BasestationController import BasestationController
from Robot.RobotController import RobotController

if __name__ == "__main__":
    try: 
        if sys.argv[1][-3:] == '.py':   # If the first arugments ends with .py -> a config file is entered
            config_path = sys.argv[1]   # Extract the path
            python_file = config_path.split('/')[-1].replace('.py', '')     # Extract the module name (python file name)
            spec = importlib.util.spec_from_file_location(python_file, config_path) 
            config_file = importlib.util.module_from_spec(spec)
            sys.modules[python_file] = config_file
            spec.loader.exec_module(config_file)

            # TODO: verbose
            # ProcessProcedure.verbose = true

            if hasattr(config_file, 'BasestationConfig'):
                base_config = config_file.BasestationConfig()       # Instantiate config from injected file
                ConfigValidator.validate_base_config(base_config)   # Validate config as a BasestationConfig
                BasestationController(base_config).do_experiment()  # Instantiate controller with config and start experiment
            elif hasattr(config_file, 'RobotConfig'):
                robot_config = config_file.RobotConfig()
                ConfigValidator.validate_robot_config(robot_config)
                RobotController(robot_config)
            else:
                raise ConfigInvalidClassNameError
        else:                           # Else, a utility command is entered
            CLIRegister.parse_command(sys.argv)
    except Exception as e:
        traceback.print_exc()