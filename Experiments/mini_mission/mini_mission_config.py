from Experiments.mini_mission.modules.sensors.CameraSensor import CameraSensor
from Experiments.mini_mission.modules.recording.MetricsRecorder import MetricsRecorder
from Experiments.mini_mission.modules.movement.MovementController import MovementController
from Experiments.mini_mission.modules.movement.RotationDirection import RotationDirection
from Experiments.mini_mission.modules.sensors.OdomSensor import OdomSensor

from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController

from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType

from ProgressManager.RunTable.Models.RunProgress import RunProgress

import time
import rospy
import pandas as pd
from rospy import ServiceProxy
from std_srvs.srv import (Empty, EmptyRequest, EmptyResponse)
from rospy.timer import Rate
from geometry_msgs.msg import Twist

from typing import Dict, List
from pathlib import Path
from multiprocessing import Event
import requests

class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    name:                       str             = "mini_mission"
    # Required ROS version for this experiment to be ran with 
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = 1
    required_ros_distro:        str             = "melodic"
    # Experiment operation types
    operation_type:             OperationType   = OperationType.SEMI
    # Run settings
    run_duration_in_ms:         int             = 0
    time_between_runs_in_ms:    int             = 0
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

    service_computation_start: ServiceProxy
    service_computation_stop: ServiceProxy
    
    service_networking_start: ServiceProxy
    service_networking_stop: ServiceProxy

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""
        EventSubscriptionController.subscribe_to_multiple_events([
            (RobotRunnerEvents.START_RUN,           self.start_run),
            (RobotRunnerEvents.START_MEASUREMENT,   self.start_measurement),
            (RobotRunnerEvents.LAUNCH_MISSION,      self.launch_mission),
            (RobotRunnerEvents.STOP_MEASUREMENT,    self.stop_measurement),
            (RobotRunnerEvents.STOP_RUN,            self.stop_run),
            (RobotRunnerEvents.CONTINUE,            self.signal_continue),
            (RobotRunnerEvents.POPULATE_RUN_DATA,   self.populate_run_data)
        ])
        
        print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        run_table = RunTableModel(
            factors = [
                FactorModel("mission_task", ['computation', 'video', 'networking']),
                FactorModel("runs_per_variation", range(1, 6))
            ],
            data_columns=["avg_cpu", "avg_ram"]
        )

        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def start_run(self, context: RobotRunnerContext) -> None:
        print("Start run -- Setup!")
        rospy.init_node("robot_runner")
        self.metrics_recorder = MetricsRecorder(str(context.run_dir.absolute()) + '/metrics.txt')
        self.mvmnt_command = Twist()
        self.odom_controller = OdomSensor()
        self.camera_controller = CameraSensor()
        self.ros_rate = rospy.Rate(10)
        self.mvmnt_controller = MovementController(self.ros_rate)

        self.service_computation_start = rospy.ServiceProxy('/computation/start', Empty)
        self.service_computation_stop = rospy.ServiceProxy('/computation/stop', Empty)

        self.service_networking_start = rospy.ServiceProxy('/networking/start', Empty)
        self.service_networking_stop = rospy.ServiceProxy('/networking/stop', Empty)

    def start_measurement(self, context: RobotRunnerContext) -> None:
        self.metrics_recorder.start_recording()

    def launch_mission(self, context: RobotRunnerContext) -> RunProgress:
        def drive_forward_10_seconds():
            print("driving forwards 10 seconds")
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            self.current_heading = yaw

            start_time = time.time()
            while time.time() - start_time < 10:                
                # self.mvmnt_command.linear.x = self.mvmnt_controller.default_speed
                # self.mvmnt_command.angular.z = self.mvmnt_controller.calculate_self_steering_angular_vel(self.current_heading, yaw)

                # self.mvmnt_controller.drive(self.mvmnt_command)
                self.mvmnt_controller.drive_to_heading_with_speed(self.current_heading, 0.6)

            self.mvmnt_controller.stop()
            print("stopped driving")

        def rotate_180_degrees():
            print("rotating 180 degrees")
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            self.mvmnt_controller.turn_in_degrees(yaw, 180, RotationDirection.CLCKWISE)
            print("rotation completed")

        def perform_operation(operation):
            print("performing operation")
            if operation == 'computation':  # Calculate fibonacci for 10 seconds
                print("computing...")
                self.service_computation_start(EmptyRequest())
                time.sleep(10)
                self.service_computation_stop(EmptyRequest())
                
            elif operation == 'networking':
                print("networking...")
                self.service_networking_start(EmptyRequest())
                time.sleep(10)
                self.service_networking_stop(EmptyRequest())

            elif operation == 'video':
                print("recording...")
                self.camera_controller.start_recording()
                time.sleep(10)
                self.camera_controller.stop_recording()

            print("operation performed")
        
        variation = context.run_variation
        operation = variation['mission_task'] # Factor name can be used as key, values are treatments: computation, networking, streaming

        print("Called config during run!")
        # =========== MISSION =========== 
        time.sleep(5)

        drive_forward_10_seconds()
        if operation == 'video':
            self.camera_controller.spawn()  # Camera is going to be needed the next few steps

        time.sleep(5)

        perform_operation(operation)    # 10 seconds
        time.sleep(5)
        perform_operation(operation)    # 10 seconds
        time.sleep(5)
        perform_operation(operation)    # 10 seconds

        if operation == 'video':
            self.camera_controller.despawn() # Camera is no longer needed
        time.sleep(5)

        rotate_180_degrees()
        drive_forward_10_seconds()

        time.sleep(5)
        # =========== MISSION =========== 

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        self.metrics_recorder.stop_recording()

    def stop_run(self, context: RobotRunnerContext) -> None:
        self.mvmnt_controller.stop()

    def signal_continue(self) -> None:
        input('\n\n>> To continue with the next run, press ENTER. <<\n\n')

    def populate_run_data(self, context: RobotRunnerContext) -> dict:
        variation = context.run_variation
        metrics_location = str(context.run_dir.absolute()) + '/metrics.txt'

        df = pd.read_csv(metrics_location, sep=",", header=None)
        df.columns = ["cpu", "ram", "bat"]
        df = df[df.cpu != 0]
        df = df[df.ram != 0]
        df = df[df.bat != 0]

        variation['avg_cpu'] = df['cpu'].mean()
        variation['avg_ram'] = df['ram'].mean()

        return variation

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
