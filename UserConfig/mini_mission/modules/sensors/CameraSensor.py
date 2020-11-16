import rospy
from rospy import ServiceProxy
from std_srvs.srv import (Empty, EmptyRequest, EmptyResponse)

from Backbone.Architecture.Singleton import Singleton

class CameraSensor(metaclass=Singleton):
    __start_service_proxy: ServiceProxy
    __stop_service_proxy: ServiceProxy

    def __init__(self):
        self.__start_service_proxy = rospy.ServiceProxy('/camera/start', Empty)
        self.__stop_service_proxy = rospy.ServiceProxy('/camera/stop', Empty)

    def start_recording(self) -> None:
        self.__start_service_proxy(EmptyRequest())

    def stop_recording(self):
        self.__stop_service_proxy(EmptyRequest())