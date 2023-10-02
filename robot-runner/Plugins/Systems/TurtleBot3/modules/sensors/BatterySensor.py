import rclpy
from sensor_msgs.msg import BatteryState

from ExperimentOrchestrator.Architecture.Singleton import Singleton
import getpass
import subprocess

class BatterySensor(metaclass=Singleton):
    def __init__(self):
        self.__battery_percentage = 0.0
        self.node = rclpy.create_node('battery_sensor')
        self.__battery_sub = self.node.create_subscription(BatteryState, '/battery_state', self.__battery_callback, 10)

    def __battery_callback(self, msg: BatteryState):
        self.__battery_percentage = msg.percentage

    def get_percentage(self) -> float:
        # read file /home/roy/project/robot-runner/energy_data.txt the last line
        cpu_energy = 0.0
        with open("/home/roy/energy_data.txt", "r") as f:
            powerjoular_output = f.readlines()[-1]
            cpu_energy = float(powerjoular_output.split(",")[2])
        self.__battery_percentage = cpu_energy
        return self.__battery_percentage


