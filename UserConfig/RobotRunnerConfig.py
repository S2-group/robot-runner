from Backbone.Events.Models.RobotRunnerEvents import RobotRunnerEvents
from Backbone.Events.EventSubscriptionController import EventSubscriptionController
from Backbone.ExperimentOutput.Models.ExperimentModel import ExperimentModel
from Backbone.ExperimentOutput.Models.ExperimentFactorModel import ExperimentFactorModel
from Backbone.Config.Models.RobotRunnerContext import RobotRunnerContext
from Backbone.Config.Models.OperationType import OperationType

from typing import Dict, List
from pathlib import Path
from multiprocessing import Event

class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    name:                       str             = "new_robot_runner_experiment"
    # Required ROS version for this experiment to be ran with 
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = 2
    required_ros_distro:        str             = "foxy"
    # Experiment operation types
    operation_type:             OperationType   = OperationType.AUTO
    # Run settings
    run_duration_in_ms:         int             = 5000
    time_between_runs_in_ms:    int             = 1000
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path             = Path("~/Documents/experiments")
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""

        EventSubscriptionController.subscribe_to_multiple_events([ 
            (RobotRunnerEvents.BEFORE_EXPERIMENT, self.before_experiment), 
            (RobotRunnerEvents.START_RUN, self.start_run),
            (RobotRunnerEvents.START_MEASUREMENT, self.start_measurement),
            # (RobotRunnerEvents.DURING_RUN, self.during_run),  # UNSUPPORT FOR NOW (Multiprocessing error)
            (RobotRunnerEvents.STOP_MEASUREMENT, self.stop_measurement),
            (RobotRunnerEvents.STOP_RUN, self.stop_run),
            (RobotRunnerEvents.POPULATE_RUN_DATA, self.populate_run_data),
            (RobotRunnerEvents.AFTER_EXPERIMENT, self.after_experiment)
        ])
        
        print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        """Create and return the run_table here. A run_table is a List (rows) of tuples (columns), 
        representing each run robot-runner must perform"""
        experiment_model = ExperimentModel(
            treatments = [
                ExperimentFactorModel("example_factor", ['example_treatment1', 'example_treatment2'])
            ],
            exclude_variations = [
                {"example_treatment1"},     # all runs having treatment example_treatment1 will be excluded
                {"example_treatment1", "example_treatment2"} # all runs having the combination <treatment1, treatment2> will be excluded
            ] 
        )
        experiment_model.create_experiment_run_table()
        return experiment_model.get_experiment_run_table()

    def before_experiment(self) -> None:
        """Perform any activity required before starting the experiment here"""

        print("Config.before_experiment() called!")

    def start_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting the run here. 
        Activities before and after starting the run should also be performed here."""
        
        print("Config.start_run() called!")

    def start_measurement(self, context: RobotRunnerContext) -> None:
        print("Config.start_measurement called!")

    def during_run(self, context: RobotRunnerContext, run_completed_event: Event) -> None:
        """Perform any activity interacting with the robotic
        system in question (simulated or real-life) here."""
        # Signalling the run has been completed
        # If run_duration_in_ms has been set to 0; use this signal to let robot-runner know the run has been completed.
        # If run_duration_in_ms has been set > 0; use this signal to prematurely stop the run (do not wait until run_duration has been elapsed)
        # NOTE: Remove / comment out this line if you do not want to prematurely stop a run in case run_duration_in_ms has been set.

        print("Config.during_run() called!")
        # run_completed_event.set()     # UNCOMMENT THIS IF YOU WANT TO PREMATURELY KILL THE RUN (run_duration_in_ms > 0)
                                        # UNCOMMENT THIS IF YOU WANT TO KILL THE RUN PROGRAMATICALLY (run_duration_in_ms == 0)

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        print("Config.stop_measurement called!")

    def stop_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for stopping the run here.
        Activities before and after stopping the run should also be performed here."""
        
        print("Config.stop_run() called!")
    
    def populate_run_data(self, context: RobotRunnerContext) -> tuple:
        """Return the run data as a row for the output manager represented as a tuple"""
        return None

    def after_experiment(self) -> None:
        """Perform any activity required after stopping the experiment here"""

        print("Config.after_experiment() called!")

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
