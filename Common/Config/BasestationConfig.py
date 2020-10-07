from enum import Enum
from typing import List
from pathlib import Path

class OperationType(Enum):
    AUTO = 1
    SEMI = 2

class BasestationConfig:
    # =================================================USER SPECIFIED CONFIG=================================================
    # Name for this experiment
    name:                       str             = "new_robot_runner_experiment"
    # Required ROS version for this experiment to be ran with 
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = 2
    required_ros_distro:        str             = "foxy"
    # Use simulator or not (gazebo)
    use_simulator:              bool            = True
    # Experiment operation types
    operation_type:             OperationType   = OperationType.AUTO
    # Run settings
    number_of_runs:             int             = 2
    run_duration_in_ms:         int             = 5000
    time_between_runs_in_ms:    int             = 1000
    # Start run criteria (ROS)
    nodes_must_be_available:    List[str]       = ["/example_node1", "/example_node2"]
    topics_must_be_available:   List[str]       = ["/example_topic", "/example_topic2"]
    services_must_be_available: List[str]       = ["/example_service", "/example_service2"]
    # ROS Recording settings
    topics_to_record:           List[str]       = ["/record_topic", "/record_topic2"]
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path             = Path("~/Documents/experiments")

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    def __init__(self):
        """Executes immediately after program start, on config load"""

    def execute_script_before_experiment(self) -> None:
        """Executes before the first run of the experiment"""
        pass

    def execute_script_before_run(self) -> None:
        """Executes before every run of the experiment"""
        pass

    def execute_script_after_launch(self) -> None:
        """Executes after every run is started but before measurement starts"""
        pass

    def execute_script_interaction(self) -> None:
        """Executes during the measurement window"""
        pass

    def execute_script_before_close(self) -> None:
        """Executes before every run is ended"""
        pass

    def execute_script_after_run(self) -> None:
        """Executes after a run completes"""
        pass
    
    def execute_script_after_experiment(self) -> None:
        """Executes after the last run of the experiment completes"""
        pass

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
