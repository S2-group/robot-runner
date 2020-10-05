import inspect
from Common.Misc.PathValidation import is_path_exists_or_creatable_portable

from Common.Config.RobotConfig import RobotConfig
from Common.Config.BasestationConfig import (BasestationConfig, OperationType)

from Common.Procedures.ProcessProcedure import ProcessProcedure
from Common.Procedures.OutputProcedure import OutputProcedure as output

from Common.CustomErrors.CustomErrors import ConfigInvalidError

class ConfigValidator:
    @staticmethod
    def __check_expression(name, value, expected, expression):
        if expression(value, expected):
            raise ConfigInvalidError(name, value, expected)

    @staticmethod
    def basestation_validate(config: BasestationConfig):
        # Display config and its values in a user-friendly manner before initialising the experiment
        output.console_log_tabulate_class(config)

        # Mandatory fields check
        installed_ros_version = str(ProcessProcedure.subprocess_check_output(['printenv','ROS_DISTRO']).strip()) \
                                    .replace('\n', '') \
                                    .replace('b', '') \
                                    .replace("'", "")

        # required_ros_version
        ConfigValidator.__check_expression('required_ros_version', config.required_ros_version, installed_ros_version,
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
                    "every item must start with '/' example: /ros_example", 
                    (lambda a, b: '/' not in item)
                )

        check_str_collections("nodes_must_be_available", config.nodes_must_be_available)
        check_str_collections("topics_must_be_available", config.topics_must_be_available)
        check_str_collections("services_must_be_available", config.services_must_be_available)
        check_str_collections("topics_to_record", config.topics_to_record)

        ConfigValidator.__check_expression("results_output_path", config.results_output_path,
            "path must be valid and writable",
            (lambda a, b: is_path_exists_or_creatable_portable(a))
        ) 

        # gazebo_version =

        # if s


    @staticmethod
    def robot_validate(config: RobotConfig):
        pass
