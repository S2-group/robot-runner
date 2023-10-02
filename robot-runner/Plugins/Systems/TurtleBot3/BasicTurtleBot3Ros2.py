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
import os
import signal
import subprocess
from nav_msgs.msg import Odometry
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
        self.world_process = None
        self.cartographer_process = None
        self.navigation_process = None
        self.odom_process = None
        self.odom_msg_counts = 0


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
        # command = "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
        # # run the command
        # self.world_process = subprocess.Popen("exec "+command, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        # time.sleep(10)

    def launch_mini_mission_real_world(self, context: RobotRunnerContext):
        # cartographer_command = "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"
        # navi_command = "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml"
        # hz_command = "ros2 topic hz /odom > ~/odom.txt"
        # # run the command
        # self.cartographer_process = subprocess.Popen("exec "+cartographer_command, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        # self.navigation_process = subprocess.Popen("exec "+navi_command, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        # # self.odom_process = subprocess.Popen("exec "+hz_command, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        # time.sleep(20)
        send_goal_command = 'ros2 action send_goal navigate_to_pose nav2_msgs/NavigateToPose "pose: { header: {frame_id: "map"}, pose: {position: {x: 33.4, y: 35.8, z: 0}} }"'
        subprocess.Popen("exec "+send_goal_command, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        time.sleep(20)
    def start_measurement_mission(self):
        if self.metrics_recorder:
            self.metrics_recorder.start_recording()

    def odom_callback(self, msg: Odometry):
        self.odom_msg_counts += 1


    def stop_measurement_mission(self):
        if self.metrics_recorder:
            self.metrics_recorder.stop_recording()

    def stop_run_mission(self):
        # if self.mvmnt_controller:
        time.sleep(5)
        # self.world_process.send_signal(signal.SIGINT)
        # self.cartographer_process.send_signal(signal.SIGINT)
        # self.navigation_process.send_signal(signal.SIGINT)
        # self.odom_msg_counts = 0
        # time.sleep(5)
        # kill_cmd_1 = "killall --regexp 'rviz' "
        # kill_cmd_2 = "killall --regexp 'ros2' "
        # subprocess.Popen("exec "+kill_cmd_1, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        # subprocess.Popen("exec "+kill_cmd_2, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        # self.world_process.wait()
        # self.cartographer_process.wait()
        # self.navigation_process.wait()
        # self.world_process.kill()
        # self.cartographer_process.kill()
        # self.navigation_process.kill()
        pass  # Stop the movement controller