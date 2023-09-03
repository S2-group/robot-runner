import rclpy
from std_msgs.msg import Float64
import psutil

from ExperimentOrchestrator.Architecture.Singleton import Singleton


class CPUSensor(metaclass=Singleton):
    def __init__(self):
        self.__cpu_percentage = 0.0
        self.node = rclpy.create_node('cpu_sensor')
        self.__cpu_sub = self.node.create_subscription(Float64, '/cpu_usage', self.__cpu_usage_clbk, 10)

    def __cpu_usage_clbk(self, msg: Float64):
        self.__cpu_percentage = msg.data

    def get_percentage(self) -> float:
        self.__cpu_percentage = psutil.cpu_percent(interval=0.0)
        print("CPU usage: " + str(self.__cpu_percentage))
        return self.__cpu_percentage


