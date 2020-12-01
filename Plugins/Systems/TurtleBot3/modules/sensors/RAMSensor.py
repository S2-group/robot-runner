import rospy
from std_msgs.msg import Float64
from rospy import Subscriber

from Backbone.Architecture.Singleton import Singleton

class RAMSensor(metaclass=Singleton):
    __ram_sub: Subscriber
    __ram_percentage: float = 0.0

    def __init__(self):
        self.__ram_sub = rospy.Subscriber('/ram_usage', Float64, self.__ram_usage_clbk)

    def __ram_usage_clbk(self, msg: Float64):
        self.__ram_percentage = msg.data

    def get_percentage(self) -> float:
        return self.__ram_percentage