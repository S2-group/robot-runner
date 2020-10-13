from enum import Enum
from typing import List
from pathlib import Path
from multiprocessing import Event

class RobotRunnerContext:
    run_nr:  int
    run_dir: Path

    def __init__(self, run_nr: int, run_dir: Path):
        self.run_nr = run_nr
        self.run_dir = run_dir

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
    # ROS Recording settings
    topics_to_record:           List[str]       = ["/record_topic", "/record_topic2"]
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path             = Path("~/Documents/experiments")

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""

        print("Custom config loaded")

    def execute_script_before_experiment(self, experiment_started_event: Event) -> None:
        """Perform any activity required before starting the experiment here"""

        print("Config.execute_script_before_experiment() called!")
        experiment_started_event.set() # Signal experiment has been started successfully

    def execute_script_start_run(self, context: RobotRunnerContext, run_started_event: Event) -> None:
        """Perform any activity required for starting the run here. 
        Activities before and after starting the run should also be performed here."""
        
        print("Config.execute_script_start_run() called!")
        run_started_event.set() # Signal run has been started successfully

    def execute_script_interaction_during_run(self, context: RobotRunnerContext, run_completed_event: Event) -> None:
        """Perform any activity interacting with the robotic
        system in question (simulated or real-life) here."""
        # Signalling the run has been completed
        # If run_duration_in_ms has been set to 0; use this signal to let robot-runner know the run has been completed.
        # If run_duration_in_ms has been set > 0; use this signal to prematurely stop the run (do not wait until run_duration has been elapsed)
        # NOTE: Remove / comment out this line if you do not want to prematurely stop a run in case run_duration_in_ms has been set.

        print("Config.execute_script_interaction_during_run() called!")
        run_completed_event.set()

    def execute_script_stop_run(self, context: RobotRunnerContext, run_stopped_event: Event) -> None:
        """Perform any activity required for stopping the run here.
        Activities before and after stopping the run should also be performed here."""
        
        print("Config.execute_script_stop_run() called!")
        run_stopped_event.set() # Signal run has been stopped successfully
    
    def execute_script_after_experiment(self, experiment_ended_event: Event) -> None:
        """Perform any activity required after stopping the experiment here"""

        print("Config.execute_script_after_experiment() called!")
        experiment_ended_event.set() # Signal the experiment has been ended successfully

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
