import rclpy
from sensor_msgs.msg import BatteryState

from ExperimentOrchestrator.Architecture.Singleton import Singleton


class BatterySensor(metaclass=Singleton):
    def __init__(self):
        self.__battery_percentage = 0.0
        self.node = rclpy.create_node('battery_sensor')
        self.__battery_sub = self.node.create_subscription(BatteryState, '/battery_state', self.__battery_callback, 10)

    def __battery_callback(self, msg: BatteryState):
        self.__battery_percentage = msg.percentage

    def get_percentage(self) -> float:
        return self.__battery_percentage


