import rospy
from rospy import Service
from std_srvs.srv import (Empty, EmptyRequest, EmptyResponse)

class ClientOnDemandComponent:
    __component_name: str
    __is_spawned: bool

    __change_event_method = None

    __spawn_service: Service
    __despawn_service: Service

    def __init__(self, component_name: str, change_event):
        self.__is_spawned = False
        self.__component_name = component_name
        self.__change_event_method = change_event

        self.__spawn_service = rospy.Service(component_name + '/spawn', Empty, self.__spawn_clbk)
        self.__despawn_service = rospy.Service(component_name + '/despawn', Empty, self.__despawn_clbk)

    def __spawn_clbk(self, msg: EmptyRequest) -> EmptyResponse:
        if not self.__is_spawned:
            self.__is_spawned = True
            self.__change_event_method()

        return EmptyResponse()

    def __despawn_clbk(self, msg: EmptyRequest) -> EmptyResponse:
        if self.__is_spawned:
            self.__is_spawned = False
            self.__change_event_method()

        return EmptyResponse()

    def is_spawned(self) -> bool:
        return self.__is_spawned