import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext

# Update the following imports based on your ROS2 package structure
# from your_package.modules.sensors.CameraSensor import CameraSensor
from Plugins.Systems.TurtleBot3.modules.recording.MetricsRecorder import MetricsRecorder
# from your_package.modules.movement.MovementController import MovementController
# from your_package.modules.movement.RotationDirection import RotationDirection
# from your_package.modules.sensors.OdomSensor import OdomSensor

class BasicTurtleBot3Ros2(Node):
    def __init__(self):
        # super().__init__('robot_runner')
        rclpy.init(args=None)
        self.node = rclpy.create_node('robot_runner')
        # self.metrics_recorder = MetricsRecorder(str(context.run_dir.absolute()) + '/metrics.txt')
        self.mvmnt_controller = None
        self.mvmnt_command = Twist()
        self.service_computation_start = self.node.create_client(Empty, '/computation/start')
        self.service_computation_stop = self.node.create_client(Empty, '/computation/stop')
        self.service_networking_start = self.node.create_client(Empty, '/networking/start')
        self.service_networking_stop = self.node.create_client(Empty, '/networking/stop')

        # Wait for service clients to be available
        # while not self.service_computation_start.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('computation/start service not available, waiting again...')
        # while not self.service_computation_stop.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('computation/stop service not available, waiting again...')
        # while not self.service_networking_start.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('networking/start service not available, waiting again...')
        # while not self.service_networking_stop.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('networking/stop service not available, waiting again...')

        # self.service_computation_start = self.create_client(Empty, '/computation/start')
        # self.service_computation_stop = self.create_client(Empty, '/computation/stop')
        # self.service_networking_start = self.create_client(Empty, '/networking/start')
        # self.service_networking_stop = self.create_client(Empty, '/networking/stop')

    def start_run_mini_mission_real_world(self, context: RobotRunnerContext):
        # Initialize the ROS2 node
        self.metrics_recorder = MetricsRecorder(str(context.run_dir.absolute()) + '/metrics.csv')

    def start_measurement_mission(self):
        if self.metrics_recorder:
            self.metrics_recorder.start_recording()

    def stop_measurement_mission(self):
        if self.metrics_recorder:
            self.metrics_recorder.stop_recording()

    def stop_run_mission(self):
        # if self.mvmnt_controller:
        pass  # Stop the movement controller