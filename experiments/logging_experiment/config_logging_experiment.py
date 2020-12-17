import rospy
from rospy.topics import Subscriber
from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType

from Plugins.Profilers.INA219Profiler import INA219Profiler
from Plugins.Profilers.NetworkProfiler import NetworkProfiler
from Plugins.Systems.TopicSubscriber import TopicSubscriber

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, LaserScan, Imu, JointState, MagneticField
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage

from typing import Dict, List
from pathlib import Path

import time

class RobotRunnerConfig:
    # =================================================USER SPECIFIC NECESSARY CONFIG=================================================
    # Name for this experiment
    name:                       str             = "logging_experiment"
    # Required ROS version for this experiment to be ran with 
    # NOTE: (e.g. ROS2 foxy or eloquent)
    # NOTE: version: 2
    # NOTE: distro: "foxy"
    required_ros_version:       int             = 1
    required_ros_distro:        str             = "melodic"
    # Experiment operation types
    operation_type:             OperationType   = OperationType.SEMI
    # Run settings
    time_between_runs_in_ms:    int             = 1000
    # Path to store results at
    # NOTE: Path does not need to exist, will be appended with 'name' as specified in this config and created on runtime
    results_output_path:        Path             = Path("~/Documents/experiments")
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    ina219_profiler: INA219Profiler = None
    network_profiler: NetworkProfiler = None
    topic_subscriber: TopicSubscriber = None
    topic_handlers: List[Subscriber] = None

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""

        EventSubscriptionController.subscribe_to_multiple_events([ 
            (RobotRunnerEvents.BEFORE_RUN,          self.before_run),
            (RobotRunnerEvents.START_RUN,           self.start_run),
            (RobotRunnerEvents.START_MEASUREMENT,   self.start_measurement),
            (RobotRunnerEvents.LAUNCH_MISSION,      self.launch_mission),
            (RobotRunnerEvents.STOP_MEASUREMENT,    self.stop_measurement),
            (RobotRunnerEvents.CONTINUE,            self.continue_to_next_run)
        ])
        
        print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        """Create and return the run_table here. A run_table is a List (rows) of tuples (columns), 
        representing each run robot-runner must perform"""
        run_table = RunTableModel(
            factors = [
                FactorModel("logging", ['ON', 'OFF']),
                FactorModel("mission_type", ['computation', 'networking']),
                FactorModel("run_number", range(1, 7))
            ]
        )
        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def before_run(self) -> None:
        input('\n\n>> To start run, press ENTER. <<\n\n')

    def start_run(self, context: RobotRunnerContext) -> None:
        self.ina219_profiler = INA219Profiler("/media/swanborn/SD_CARD/DATA.txt")
        self.network_profiler = NetworkProfiler()
        self.topic_subscriber = TopicSubscriber()

    def start_measurement(self, context: RobotRunnerContext) -> None:
        self.network_profiler.start_sniffing_to_file_between_robot_and_remotepc(
            context=context, 
            network_interface='wlp5s0',
            robot_ip_addr='10.42.0.110', 
            remotepc_ip_addr='10.42.0.1', 
            output_file_name='wireshark_dump.pcap'
        )

    def launch_mission(self, context: RobotRunnerContext) -> None:
        def empty_callback_handler(data):
            print(f"topic callback handled: {str(data)}", sep=' ', end='', flush=True)

        logging = context.run_variation['logging']
        if logging == 'ON':
            topics_datatypes_map = {
                '/cmd_vel': Twist,
                '/battery_state': BatteryState,
                '/odom': Odometry,
                '/scan': LaserScan,
                '/diagnostics': DiagnosticArray,
                '/imu': Imu,
                '/joint_states': JointState,
                '/magnetic_field': MagneticField,
                '/tf': TFMessage
            }

            self.topic_handlers = self.topic_subscriber.subscribe_to_multiple_topics_on_one_callback(
                topic_datatype_map=topics_datatypes_map, 
                callback=empty_callback_handler
            )

        print("waiting for experiment to finish...")
        time.sleep(80)
        
    def stop_measurement(self, context: RobotRunnerContext) -> None:
        self.network_profiler.stop_sniffing()

        logging = context.run_variation['logging']
        if logging == 'ON':
            self.topic_subscriber.unregister_from_multiple_subscriptions(self.topic_handlers)

        energy_data = self.ina219_profiler.halt_and_collect_measures_from_sd_card_and_return_data()
        self.ina219_profiler.move_data_file_to_run_folder(context, 'energy_data.txt')

    def continue_to_next_run(self) -> None:
        input('\n\n>> To continue with the next run, press ENTER. <<\n\n')

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
