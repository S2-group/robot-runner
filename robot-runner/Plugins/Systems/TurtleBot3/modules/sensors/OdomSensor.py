import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from squaternion import Quaternion
from ExperimentOrchestrator.Architecture.Singleton import Singleton
from typing import Tuple

class OdomSensor(metaclass=Singleton):
    def __init__(self):
        self.node = rclpy.create_node('odom_sensor')
        self.__odom_sub = self.node.create_subscription(Odometry, '/odom', self.__odom_callback, 10)
        self.__roll = self.__pitch = self.__yaw = 0.0

    def __odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        q = Quaternion(orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z)
        (self.__roll, self.__pitch, self.__yaw) = q.to_euler()
        # VERBOSE: print(self.__roll, self.__pitch, self.__yaw)

    def get_odometry_as_tuple(self) -> Tuple[float, float, float]:
        return (self.__roll, self.__pitch, self.__yaw)
