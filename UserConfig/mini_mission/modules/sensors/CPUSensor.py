import rospy
from std_msgs.msg import Float64
from rospy import Subscriber

from Backbone.Architecture.Singleton import Singleton

class CPUSensor(metaclass=Singleton):
    __cpu_sub: Subscriber
    __cpu_percentage: float = 0.0

    def __init__(self):
        self.__cpu_sub = rospy.Subscriber('/cpu_usage', Float64, self.__cpu_usage_clbk)

    def __cpu_usage_clbk(self, msg: Float64):
        self.__cpu_percentage = msg.data

    def get_percentage(self) -> float:
        return self.__cpu_percentage