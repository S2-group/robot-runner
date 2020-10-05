import sys
import importlib.machinery
from Common.CLIRegister.CLIRegister import CLIRegister
from Common.CustomErrors.CustomErrors import (CommandNotRecognisedError, ConfigInvalidClassNameError)
from Common.Procedures.OutputProcedure import OutputProcedure as output

from Common.Config.Validation.ConfigValidator import ConfigValidator

from Basestation.Basestation import Basestation
from Robot.Robot import Robot

if __name__ == "__main__":
    try: 
        if sys.argv[1][-3:] == '.py':   # If the first arugments ends with .py -> a config file is entered
            config_path = sys.argv[1]   # Extract the path
            python_file = config_path.split('/')[-1].replace('.py', '')     # Extract the module name (python file name)
            config_file = importlib.machinery.SourceFileLoader(python_file, config_path).load_module()   # Load the python module

            # TODO: verbose
            # ProcessProcedure.verbose = true

            if hasattr(config_file, 'BasestationConfig'):
                base_config = config_file.BasestationConfig()
                ConfigValidator.basestation_validate(base_config)
                Basestation(base_config)
            elif hasattr(config_file, 'RobotConfig'):
                Robot(config_file.RobotConfig())
            else:
                raise ConfigInvalidClassNameError
        else:                           # Else, a utility command is entered
            CLIRegister.parse_command(sys.argv)
    except Exception as e:
        print(e)