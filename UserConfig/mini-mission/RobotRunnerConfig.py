from Backbone.Procedures.ProcessProcedure import ProcessProcedure
from Backbone.Events.Models.RobotRunnerEvents import RobotRunnerEvents
from Backbone.Events.EventSubscriptionController import EventSubscriptionController
from Backbone.ExperimentOutput.Models.ExperimentModel import ExperimentModel
from Backbone.ExperimentOutput.Models.ExperimentFactorModel import ExperimentFactorModel
from Backbone.Config.Models.RobotRunnerContext import RobotRunnerContext
from Backbone.Config.Models.OperationType import OperationType

import time
from typing import Dict, List
from pathlib import Path
from multiprocessing import Event

class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    name:                       str             = "robot_runner_mini_mission"
    # Required ROS version for this experiment to be ran with 
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = 1
    required_ros_distro:        str             = "melodic"
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
            (RobotRunnerEvents.START_RUN, self.start_run),
            (RobotRunnerEvents.START_MEASUREMENT, self.start_measurement),
            (RobotRunnerEvents.DURING_RUN, self.during_run),
            (RobotRunnerEvents.STOP_MEASUREMENT, self.stop_measurement),
            (RobotRunnerEvents.STOP_RUN, self.stop_run),
            (RobotRunnerEvents.POPULATE_RUN_DATA, self.populate_run_data)
        ])
        
        print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        experiment_model = ExperimentModel(
            treatments = [
                ExperimentFactorModel("mission_task", ['computation', 'streaming', 'networking']),
                ExperimentFactorModel("runs_per_variation", range(1, 6))
            ]
        )

        experiment_model.create_experiment_run_table()
        return experiment_model.get_experiment_run_table()

    def start_run(self, context: RobotRunnerContext) -> None:
        print("Config.start_run() called!")

    def start_measurement(self, context: RobotRunnerContext) -> None:
        print("Config.start_measurement called!")

    def during_run(self, context: RobotRunnerContext, run_completed_event: Event) -> None:
        def perform_operation(operation):
            if operation == 'computation':
                pass    # 10 seconds of computation
            elif operation == 'networking':
                pass    # 10 seconds of networking
            elif operation == 'streaming':
                pass    # 10 seconds of streaming
            else:
                return # error!
        
        variation = context.run_variation
        operation = variation[2] # computation, networking, streaming

        # =========== MISSION =========== 
        time.sleep(5)

        # self.__cmd_pub.publish(cmd) # drive forward for 10 seconds
        # self.stop()

        time.sleep(5)
        perform_operation(operation)    # 10 seconds
        time.sleep(5)
        perform_operation(operation)    # 10 seconds
        time.sleep(5)
        perform_operation(operation)    # 10 seconds
        time.sleep(5)

        # rotate 180 degrees
        # self.__cmd_pub.publish(cmd) # drive forward for 10 seconds
        # self.stop()
        time.sleep(5)
        # =========== MISSION =========== 

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        print("Config.stop_measurement called!")

    def stop_run(self, context: RobotRunnerContext) -> None:
        print("Config.stop_run() called!")
    
    def populate_run_data(self, context: RobotRunnerContext) -> tuple:
        return None

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
