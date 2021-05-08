import rospy
from rospy.topics import Subscriber
from sensor_msgs.msg import BatteryState

from ExperimentOrchestrator.Architecture.Singleton import Singleton

class BatterySensor(metaclass=Singleton):
    __battery_sub: Subscriber
    __battery_percentage: float = 0.0

    def __init__(self):
        self.__battery_sub = rospy.Subscriber('/battery_state', BatteryState, self.__battery_callback)

    def __battery_callback(self, msg: BatteryState):
        self.__battery_percentage = msg.percentage

    def get_percentage(self) -> float:
        return self.__battery_percentage