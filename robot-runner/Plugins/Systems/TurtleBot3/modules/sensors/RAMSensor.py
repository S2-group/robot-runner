import rclpy
from std_msgs.msg import Float64
import psutil

from ExperimentOrchestrator.Architecture.Singleton import Singleton


class RAMSensor(metaclass=Singleton):
    def __init__(self):
        self.__ram_percentage = 0.0
        self.node = rclpy.create_node('ram_sensor')
        self.__ram_sub = self.node.create_subscription(Float64, '/ram_usage', self.__ram_usage_clbk, 10)

    def __ram_usage_clbk(self, msg: Float64):
        self.__ram_percentage = msg.data

    def get_percentage(self) -> float:
        self.__ram_percentage = psutil.virtual_memory().percent
        print("RAM usage: " + str(self.__ram_percentage))
        return self.__ram_percentage
