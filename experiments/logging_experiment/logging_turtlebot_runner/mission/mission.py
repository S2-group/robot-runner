import time
import requests
import threading

from rospy import Rate
from geometry_msgs.msg import Twist

from modules.movement.MovementController import MovementController
from modules.movement.RotationDirection import RotationDirection
from modules.sensors.OdomSensor import OdomSensor

class Mission:
    __is_computing: bool = False
    __is_networking: bool = False

    __operation: str

    __computation_thread: threading.Thread
    __networking_thread: threading.Thread

    ros_rate: Rate
    odom_controller: OdomSensor
    mvmnt_controller: MovementController
    mvmnt_command: Twist

    n1, n2 = 0, 1

    def __init__(self, operation: str):        
        self.mvmnt_command = Twist()
        self.odom_controller = OdomSensor()
        self.ros_rate = Rate(10)
        self.mvmnt_controller = MovementController(self.ros_rate)
        self.__operation = operation

    def perform_mission(self):
        def drive_forward_10_seconds():
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            self.current_heading = yaw

            start_time = time.time()
            while time.time() - start_time < 10:                
                self.mvmnt_controller.drive_to_heading_with_speed(self.current_heading, 0.6)

            self.mvmnt_controller.stop()

        def rotate_180_degrees():
            roll, pitch, yaw = self.odom_controller.get_odometry_as_tuple()
            self.mvmnt_controller.turn_in_degrees(yaw, 180, RotationDirection.CLCKWISE)

        def perform_operation(operation):
            if operation == 'computation':
                self.__computation_start()
                time.sleep(10)
                self.__computation_stop()
                
            elif operation == 'networking':
                self.__networking_start()
                time.sleep(10)
                self.__networking_stop()

        # =========== MISSION =========== 
        drive_forward_10_seconds()

        time.sleep(5)

        perform_operation(self.__operation)    # 10 seconds
        time.sleep(5)
        perform_operation(self.__operation)    # 10 seconds
        time.sleep(5)
        perform_operation(self.__operation)    # 10 seconds

        time.sleep(5)

        rotate_180_degrees()
        drive_forward_10_seconds()
        # =========== MISSION =========== 

    def __computation_start(self):
        self.__is_computing = True
        self.__computation_thread = threading.Thread(target=self.__computation_worker_thread)
        self.__computation_thread.start()

    def __computation_stop(self):
        self.__is_computing = False
        self.__computation_thread.join()

    def __computation_worker_thread(self):
        while self.__is_computing:
            nth = self.n1 + self.n2
            self.n1 = self.n2
            self.n2 = nth

    def __networking_start(self):
        self.__is_networking = True
        self.__networking_thread = threading.Thread(target=self.__networking_worker_thread)
        self.__networking_thread.start()
    
    def __networking_stop(self):
        self.__is_networking = False
        self.__networking_thread.join()

    def __networking_worker_thread(self):
        while self.__is_networking:
            requests.get(url="http://s2group.cs.vu.nl/")
