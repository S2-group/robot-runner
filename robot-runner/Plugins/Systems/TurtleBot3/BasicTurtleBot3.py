import time
import rospy
from rospy import ServiceProxy
from std_srvs.srv import (Empty, EmptyRequest)
from rospy.timer import Rate
from geometry_msgs.msg import Twist

from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext

from Plugins.Systems.TurtleBot3.modules.sensors.CameraSensor import CameraSensor
from Plugins.Systems.TurtleBot3.modules.recording.MetricsRecorder import MetricsRecorder
from Plugins.Systems.TurtleBot3.modules.movement.MovementController import MovementController
from Plugins.Systems.TurtleBot3.modules.movement.RotationDirection import RotationDirection
from Plugins.Systems.TurtleBot3.modules.sensors.OdomSensor import OdomSensor

class BasicTurtleBot3:
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

    def start_run_mini_mission_real_world(self, context: RobotRunnerContext):
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

    def start_measurement_mission(self):
        self.metrics_recorder.start_recording()

    def stop_measurement_mission(self):
        self.metrics_recorder.stop_recording()

    def stop_run_mission(self):
        self.mvmnt_controller.stop()

    def launch_mini_mission_real_world(self, context: RobotRunnerContext):
        def drive_forward_10_seconds():
            print("driving forwards 10 seconds")
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            self.current_heading = yaw

            start_time = time.time()
            while time.time() - start_time < 10:                
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