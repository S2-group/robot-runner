from UserConfig.mini_mission.modules.sensors.CameraSensor import CameraSensor
from UserConfig.mini_mission.modules.recording.MetricsRecorder import MetricsRecorder
from Backbone.Events.Models.RobotRunnerEvents import RobotRunnerEvents
from Backbone.Events.EventSubscriptionController import EventSubscriptionController
from Backbone.ExperimentOutput.Models.ExperimentModel import ExperimentModel
from Backbone.ExperimentOutput.Models.ExperimentFactorModel import ExperimentFactorModel
from Backbone.Config.Models.RobotRunnerContext import RobotRunnerContext
from Backbone.Config.Models.OperationType import OperationType

from UserConfig.mini_mission.modules.movement.MovementController import MovementController
from UserConfig.mini_mission.modules.movement.RotationDirection import RotationDirection
from UserConfig.mini_mission.modules.sensors.OdomSensor import OdomSensor

import time
import rospy
from typing import Dict, List
from pathlib import Path
from multiprocessing import Event
import requests

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
    run_duration_in_ms:         int             = 0
    time_between_runs_in_ms:    int             = 5000
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path             = Path("~/Documents/experiments")
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    metrics_recorder: MetricsRecorder
    odom_controller: OdomSensor
    camera_controller: CameraSensor
    mvmnt_controller: MovementController
    mvmnt_command: Twist
    ros_rate: Rate

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""
        EventSubscriptionController.subscribe_to_multiple_events([
            (RobotRunnerEvents.START_RUN, self.start_run),
            (RobotRunnerEvents.START_MEASUREMENT, self.start_measurement),
            (RobotRunnerEvents.DURING_RUN, self.during_run),
            (RobotRunnerEvents.STOP_MEASUREMENT, self.stop_measurement),
            (RobotRunnerEvents.STOP_RUN, self.stop_run)
        ])
        
        print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        experiment_model = ExperimentModel(
            treatments = [
                ExperimentFactorModel("mission_task", ['computation', 'video', 'networking']),
                ExperimentFactorModel("runs_per_variation", range(1, 6))
            ]
        )

        experiment_model.create_experiment_run_table()
        return experiment_model.get_experiment_run_table()

    def start_run(self, context: RobotRunnerContext) -> None:
        print("Start run -- Setup!")
        self.metrics_recorder = MetricsRecorder(str(context.run_dir.absolute()) + '/metrics.txt')
        self.mvmnt_controller = Twist()
        self.odom_controller = OdomSensor()
        self.camera_controller = CameraSensor()
        self.ros_rate = rospy.Rate(10)
        self.mvmnt_controller = MovementController(self.ros_rate)

    def start_measurement(self, context: RobotRunnerContext) -> None:
        self.metrics_recorder.start_recording()

    def during_run(self, context: RobotRunnerContext, run_completed_event: Event) -> None:
        def drive_forward_10_seconds():
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            self.current_heading = yaw

            start_time = time.time()
            while time.time() - start_time < 10:
                roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()

                self.mvmnt_command.linear.x = self.mvmnt_controller.default_speed
                self.mvmnt_command.angular.z = self.mvmnt_controller.calculate_self_steering_angular_vel(self.current_heading, yaw)

            self.mvmnt_controller.stop()

        def rotate_180_degrees():
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            self.mvmnt_controller.turn_in_degrees(yaw, RotationDirection.CLCKWISE, 180)

        def perform_operation(operation):
            start_time = time.time()

            def operating():
                return time.time() - start_time < 10

            if operation == 'computation':  # Calculate fibonacci for 10 seconds
                n1, n2 = 0, 1
                while operating():
                    nth = n1 + n2
                    n1 = n2
                    n2 = nth
                
            elif operation == 'networking':
                while operating():
                    requests.get(url="https://s2group.cs.vu.nl/")

            elif operation == 'video':
                self.camera_controller.start_recording()
                time.sleep(10)
                self.camera_controller.stop_recording()
        
        variation = context.run_variation
        operation = variation[2] # computation, networking, streaming

        # =========== MISSION =========== 
        time.sleep(5)

        drive_forward_10_seconds()

        time.sleep(5)

        perform_operation(operation)    # 10 seconds
        time.sleep(5)
        perform_operation(operation)    # 10 seconds
        time.sleep(5)
        perform_operation(operation)    # 10 seconds

        time.sleep(5)

        rotate_180_degrees()
        drive_forward_10_seconds()

        time.sleep(5)
        # =========== MISSION =========== 

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        self.metrics_recorder.stop_recording()

    def stop_run(self, context: RobotRunnerContext) -> None:
        self.mvmnt_controller.stop()

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
