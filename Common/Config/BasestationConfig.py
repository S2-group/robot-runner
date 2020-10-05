from enum import Enum
from typing import List

class OperationType(Enum):
    AUTO = 1
    SEMI = 2
    SINGLE = 3

class BasestationConfig:
    # Name for this experiment
    name:                       str             = ""
    # Required ROS version for this experiment to be ran with 
    # (e.g. foxy or eloquent)
    required_ros_version:       int             = "foxy"
    # Use simulator or not (gazebo)
    use_simulator:              bool            = True
    requored_gazebo_version:    int             = 11
    # Experiment operation types
    operation_type:             OperationType   = OperationType.AUTO
    # Run settings

    number_of_runs:             int             = 2
    run_duration_in_ms:         int             = 5000
    time_between_runs_in_ms:    int             = 1000
    # Start run criteria (ROS)
    nodes_must_be_available:    List[str]       = ["/test"]
    topics_must_be_available:   List[str]       = ["/test"]
    services_must_be_available: List[str]       = []
    # ROS Recording settings
    topics_to_record:           List[str]       = []
    # Path to store results at
    results_output_path:        str             = "~/Documents"

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

    def __iter__(self):
        """NOT RECOMMENDED TO CHANGE: this aids the tabular view of the config on start"""
        # first start by grabbing the Class items
        iters = dict((x,y) for x,y in self.__dict__.items() if x[:2] != '__')

        # then update the class items with the instance items
        iters.update(self.__dict__)

        # now 'yield' through the items
        for x,y in iters.items():
            yield x,y