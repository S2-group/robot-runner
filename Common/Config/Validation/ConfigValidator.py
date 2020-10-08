import os
import inspect
from typing import List
from pathlib import Path
from tabulate import tabulate

from Common.Misc.DictConversion import class_to_dict
from Common.Misc.PathValidation import is_path_exists_or_creatable_portable

from Common.Config.RobotConfig import RobotConfig
from Common.Config.BasestationConfig import (BasestationConfig, OperationType)

from Common.Procedures.ProcessProcedure import ProcessProcedure
from Common.Procedures.OutputProcedure import OutputProcedure as output

from Common.CustomErrors.ConfigErrors import ConfigInvalidError

class ConfigValidator:
    exception_dict: dict = {}

    @staticmethod
    def __check_expression(name, value, expected, expression):
        if expression(value, expected):
            ConfigValidator.exception_dict[name] = str(ConfigValidator.exception_dict[name]) + \
                                                    f"\n\n{ConfigInvalidError(name, value, expected)}"

    @staticmethod
    def validate_base_config(config: BasestationConfig):
        # Runtime set experiment_path
        config.experiment_path = Path(str(config.results_output_path) + f"/{config.name}")
        # Convert class to dictionary with utility method
        ConfigValidator.exception_dict = class_to_dict(config)

        # Mandatory fields check
        installed_ros_version   = int(os.environ['ROS_VERSION'])
        installed_ros_distro    = ProcessProcedure.subprocess_check_output_friendly_string(['printenv','ROS_DISTRO'])

        # required_ros_version
        ConfigValidator.__check_expression('required_ros_version', installed_ros_version, config.required_ros_version,
                                (lambda a, b: a != b)
                            )
        # required_ros_distro
        ConfigValidator.__check_expression('required_ros_version', installed_ros_distro, config.required_ros_distro,
                                (lambda a, b: a != b)
                            )
        # operation_type
        ConfigValidator.__check_expression('operation_type', config.operation_type, OperationType, 
                                (lambda a, b: not isinstance(type(a), type(b)))
                            )
        # run_replications
        ConfigValidator.__check_expression('number_of_runs', config.number_of_runs, int,
                                (lambda a, b: not isinstance(a, b))
                            )
        ConfigValidator.__check_expression('number_of_runs', config.number_of_runs, 
                                "number of runs must be > 0",
                                (lambda a, b: not a > 0)
                            )
        # run_duration
        ConfigValidator.__check_expression('run_duration_in_ms', config.run_duration_in_ms, int,
                                (lambda a, b: not isinstance(a, b))
                            )
        # time_between_runs_in_ms
        ConfigValidator.__check_expression('time_between_runs_in_ms', config.time_between_runs_in_ms, int,
                                (lambda a, b: not isinstance(a, b))
                            )

        def check_str_collections(attribute_name, collection):
            for item in collection:
                ConfigValidator.__check_expression(attribute_name, item, str, (lambda a, b: not isinstance(a,b)))
                ConfigValidator.__check_expression(attribute_name, item, 
                    "must start with '/' : /ros_example", 
                    (lambda a, b: '/' not in a)
                )

        # NO LONGER NECESSARY BUT KEPT FOR POSSIBLE FUTURE RE-ACTIVATION
        # check_str_collections("nodes_must_be_available", config.nodes_must_be_available)
        # check_str_collections("topics_must_be_available", config.topics_must_be_available)
        # check_str_collections("services_must_be_available", config.services_must_be_available)

        check_str_collections("topics_to_record", config.topics_to_record)

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
                ConfigValidator.exception_dict.items(), 
                ['Key', 'Value'], 
                tablefmt="rst"
            )
        )

    @staticmethod
    def validate_robot_config(config: RobotConfig):
        pass
