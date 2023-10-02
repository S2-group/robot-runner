from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController

from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType

from ProgressManager.RunTable.Models.RunProgress import RunProgress

from Plugins.Systems.TurtleBot3.BasicTurtleBot3Ros2 import BasicTurtleBot3Ros2
from Plugins.Profilers.INA219Profiler import INA219Profiler

import pandas as pd
from typing import Dict, List
from pathlib import Path
import os
import time
class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    ROOT_DIR = Path("/home/roy/project/robot-runner/experiments/mini_mission/experiment")
    name:                       str             = "gazebo_rviz"
    if os.path.exists(ROOT_DIR / name):
        # convert time to date and time
        name = name + "_" + time.strftime("%Y%m%d-%H%M%S")

    # Required ROS version for this experiment to be ran with 
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = any
    required_ros_distro:        str             = any
    # Experiment operation types
    operation_type:             OperationType   = OperationType.AUTO
    # Run settings
    time_between_runs_in_ms:    int             = 1000
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path             = Path("/home/roy/project/robot-runner/experiments/mini_mission/experiment/")
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    turtlebot3: BasicTurtleBot3Ros2

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""
        self.turtlebot3 = BasicTurtleBot3Ros2()
        # self.ina219 = INA219Profiler('~/Documents/DATA.TXT')
        # check if name exists in results_output_path


        EventSubscriptionController.subscribe_to_multiple_events([
            (RobotRunnerEvents.START_RUN,           self.start_run),
            (RobotRunnerEvents.START_MEASUREMENT,   self.start_measurement),
            (RobotRunnerEvents.LAUNCH_MISSION,      self.launch_mission),
            (RobotRunnerEvents.STOP_MEASUREMENT,    self.stop_measurement),
            (RobotRunnerEvents.STOP_RUN,            self.stop_run),
            (RobotRunnerEvents.CONTINUE,            self.signal_continue),
            (RobotRunnerEvents.POPULATE_RUN_DATA,   self.populate_run_data)
        ])

    def create_run_table(self) -> List[Dict]:
        run_table = RunTableModel(
            factors = [
                FactorModel("mission_task", ['gazebo']),
                FactorModel("runs_per_variation", range(1, 10))
            ],
            data_columns=["avg_cpu", "avg_ram", "avg_power",]
        )

        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def start_run(self, context: RobotRunnerContext) -> None:
        self.turtlebot3.start_run_mini_mission_real_world(context)
        time.sleep(5)
        pass

    def start_measurement(self, context: RobotRunnerContext) -> None:
        self.turtlebot3.start_measurement_mission()

    def launch_mission(self, context: RobotRunnerContext) -> RunProgress:
        self.turtlebot3.launch_mini_mission_real_world(context)
        pass

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        self.turtlebot3.stop_measurement_mission()
        # energy_data = self.ina219.halt_and_collect_measures_from_sd_card_and_return_data()  # Energy data as array of arrays is available here. (27000 rows on average)
        # self.ina219.move_data_file_to_run_folder(context, 'energy_data.txt')

    def stop_run(self, context: RobotRunnerContext) -> None:
        self.turtlebot3.stop_run_mission()
        pass

    def signal_continue(self) -> None:
        input('\n\n>> To continue with the next run, press ENTER. <<\n\n')

    def populate_run_data(self, context: RobotRunnerContext) -> dict:
        variation = context.run_variation
        metrics_location = str(context.run_dir.absolute()) + '/metrics.csv'
        # create metrics file if it does not exist
        # Path(metrics_location).touch()
        df = pd.read_csv(metrics_location, sep=",", header=None)
        print(df)
        # add header to df
        df.columns = ['cpu', 'ram', 'bat']
        print(df)
        variation['avg_cpu'] = df['cpu'].mean()
        variation['avg_ram'] = df['ram'].mean()
        # variation['avg_power'] = df['bat'].mean()
        # duration = pd.to_datetime(df['timestamp'].iloc[-1]) - pd.to_datetime(df['timestamp'].iloc[0])
        # duration_in_s = duration.total_seconds()
        variation['avg_power'] = df['bat'].mean()
        # variation['msg_count'] = df['msg_count'].mean() * duration_in_s
        print(variation)

        return variation

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
