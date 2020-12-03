import os
import subprocess
from typing import List
from pathlib import Path
from tabulate import tabulate

from ExperimentOrchestrator.Misc.DictConversion import class_to_dict
from ExperimentOrchestrator.Misc.PathValidation import is_path_exists_or_creatable_portable
from ConfigValidator.Config.RobotRunnerConfig import RobotRunnerConfig
from ConfigValidator.Config.Models.OperationType import OperationType
from ConfigValidator.CustomErrors.ConfigErrors import (ConfigInvalidError, ConfigAttributeInvalidError)

class ConfigValidator:
    config_values_or_exception_dict: dict = {}
    error_found:                     bool = False

    @staticmethod
    def __check_expression(name, value, expected, expression):
        if expression(value, expected):
            ConfigValidator \
                .config_values_or_exception_dict[name] = str(ConfigValidator.config_values_or_exception_dict[name]) + \
                                                    f"\n\n{ConfigAttributeInvalidError(name, value, expected)}"
            ConfigValidator.error_found = True

    @staticmethod
    def validate_config(config: RobotRunnerConfig):

        def subprocess_check_output_friendly_string(command_in_array: List[str]):
            def subprocess_check_output(command_in_array: List[str]):
                return subprocess.check_output(command_in_array)

            return str(subprocess_check_output(command_in_array) \
                        .strip()) \
                        .replace('\n', '') \
                        .replace('b', '') \
                        .replace("'", "")

        # Runtime set experiment_path
        config.experiment_path = Path(str(config.results_output_path) + f"/{config.name}")
        if '~' in str(config.experiment_path):
            config.experiment_path = config.experiment_path.expanduser()
        
        # Convert class to dictionary with utility method
        ConfigValidator.config_values_or_exception_dict = class_to_dict(config)

        # required_ros_version
        if config.required_ros_version is not None:                                                                             # ROS is required (not None)
            try:
                installed_ros_version = int(os.environ['ROS_VERSION'])                                                          # Get system variable ROS_VERSION
            except KeyError:
                installed_ros_version = 'not_installed'                                                                         # If not installed, set to not_installed

            if config.required_ros_version is not any or installed_ros_version == 'not_installed':                              # In case a specific ROS version is required (not any) check found vs expected.
                # required_ros_version                                                                                          # In case ROS is not_installed but ROS was required (not None), check vs expected and raise error.
                ConfigValidator.__check_expression('required_ros_version', installed_ros_version, config.required_ros_version,
                                        (lambda a, b: a != b)
                                    )
            
            if config.required_ros_distro is not None:                                                                          # ROS is required (not None)
                try:
                    installed_ros_distro = subprocess_check_output_friendly_string(['printenv','ROS_DISTRO'])                   # Get system variable ROS_DISTRO
                except:
                    installed_ros_distro = 'not_installed'                                                                      # If not installed, set to not_installed

                if config.required_ros_distro is not any or installed_ros_distro == 'not_installed':                            # In case a specific ROS distribution is required (not any) check found vs expected
                    # required_ros_distro                                                                                       # In case ROS is not_installed but ROS was required (not None), check vs expected and raise error.
                    ConfigValidator.__check_expression('required_ros_version', installed_ros_distro, config.required_ros_distro,
                                            (lambda a, b: a != b)
                                        )

        # operation_type
        ConfigValidator.__check_expression('operation_type', config.operation_type, OperationType, 
                                (lambda a, b: not isinstance(type(a), type(b)))
                            )
        # time_between_runs_in_ms
        ConfigValidator.__check_expression('time_between_runs_in_ms', config.time_between_runs_in_ms, int,
                                (lambda a, b: not isinstance(a, b))
                            )

        # Results output path
        ConfigValidator.__check_expression("results_output_path", 
                            config.results_output_path,
                            Path,
                            (lambda a, b: not isinstance(a, b))
                        )

        ConfigValidator.__check_expression("results_output_path", 
                            config.experiment_path,
                            "path must be valid and writable",
                            (lambda a, b: is_path_exists_or_creatable_portable(a))
                        )

        # Display config in user-friendly manner, including potential errors found
        print(
            tabulate(
                ConfigValidator.config_values_or_exception_dict.items(), 
                ['Key', 'Value'], 
                tablefmt="rst"
            )
        )

        if ConfigValidator.error_found:
            raise ConfigInvalidError