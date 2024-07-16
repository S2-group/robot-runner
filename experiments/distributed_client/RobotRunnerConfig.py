from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType
from ExperimentOrchestrator.Architecture.Distributed.RRWebSocketServer import RRWebSocketServer

from typing import Dict, List
from pathlib import Path

class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    name:                       str             = "distributed_rr"
    required_ros_version:       int             = any
    required_ros_distro:        str             = any
    operation_type:             OperationType   = OperationType.AUTO
    time_between_runs_in_ms:    int             = 1000
    results_output_path:        Path             = Path("~/Documents/experiments")
    distributed_clients:        int              = 1
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""

        # Local callbacks
        EventSubscriptionController.subscribe_to_multiple_events_multiple_callbacks([ 
            (RobotRunnerEvents.BEFORE_EXPERIMENT,   [self.before_experiment, self.before_experiment1]), # multi callbacks sequentially
            (RobotRunnerEvents.BEFORE_RUN,          [self.before_run]),                                            # single callback
            (RobotRunnerEvents.START_RUN,           [self.start_run]),
            (RobotRunnerEvents.START_MEASUREMENT,   [self.start_measurement]),
            (RobotRunnerEvents.LAUNCH_MISSION,      [self.launch_mission]),
            (RobotRunnerEvents.STOP_MEASUREMENT,    [self.stop_measurement]),
            (RobotRunnerEvents.STOP_RUN,            [self.stop_run]),
            (RobotRunnerEvents.POPULATE_RUN_DATA,   [self.populate_run_data]),
            (RobotRunnerEvents.AFTER_EXPERIMENT,    [self.after_experiment])                           
        ])

        # Remote calls
        EventSubscriptionController.subscribe_to_multiple_events_multiple_remote_calls([
            (RobotRunnerEvents.BEFORE_EXPERIMENT, [('remote_method', ['c1'])]),
        ])

        print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        """Create and return the run_table here. A run_table is a List (rows) of tuples (columns), 
        representing each run robot-runner must perform"""
        run_table = RunTableModel(
            factors = [
                FactorModel("example_factor", ['example_treatment1', 'example_treatment2'])
            ],
            exclude_variations = [
                {"example_treatment1"},     # all runs having treatment example_treatment1 will be excluded
                {"example_treatment1", "example_treatment2"} # all runs having the combination <treatment1, treatment2> will be excluded
            ] 
        )
        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def before_experiment(self) -> None:
        """Perform any activity required before starting the experiment here"""
        print("Config.before_experiment() called!")

    def before_experiment1(self) -> None:
        print("Config.before_experiment1() called!")

    def before_run(self) -> None:
        """Perform any activity required before starting a run, no context is available 
        here as the run is not yet active (BEFORE RUN)"""
        print("Config.before_run() called!")

    # experiment runs are not implemented with async yet
    def start_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting the run here. 
        Activities before and after starting the run should also be performed here."""
        
        print("Config.start_run() called!")

    # experiment runs are not implemented with async yet
    def start_measurement(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting measurements."""
        print("Config.start_measurement called!")

    # experiment runs are not implemented with async yet
    def launch_mission(self, context: RobotRunnerContext) -> None:
        """Perform any activity interacting with the robotic
        system in question (simulated or real-life) here."""

        print("Config.launch_mission() called!")

    # experiment runs are not implemented with async yet
    def stop_measurement(self, context: RobotRunnerContext) -> None:
        """Perform any activity here required for stopping measurements."""
        print("Config.stop_measurement called!")

    # experiment runs are not implemented with async yet
    def stop_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for stopping the run here.
        Activities before and after stopping the run should also be performed here."""
        
        print("Config.stop_run() called!")

    # experiment runs are not implemented with async yet
    def populate_run_data(self, context: RobotRunnerContext) -> tuple:
        """Return the run data as a row for the output manager represented as a tuple"""
        return None

    # async call
    def after_experiment(self) -> None:
        """Perform any activity required after stopping the experiment here"""

        print("Config.after_experiment() called!")

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
