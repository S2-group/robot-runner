import os
import requests
import threading
import subprocess as sp

import rospy
from rospy import Service
from std_srvs.srv import (Empty, EmptyRequest, EmptyResponse)

from mission.camera.ClientOnDemandComponent import ClientOnDemandComponent

class Mission:
    __dir_path = None
    __camera_ondemand_controller: ClientOnDemandComponent
    __camera_process: sp.Popen

    __service_start_computation: Service
    __service_stop_computation: Service
    
    __service_start_networking: Service
    __service_stop_networking: Service

    __is_computing: bool = False
    __is_networking: bool = False

    __computation_thread: threading.Thread
    __networking_thread: threading.Thread

    n1, n2 = 0, 1

    def __init__(self):
        self.__camera_process = None
        self.__dir_path = os.path.dirname(os.path.realpath(__file__))
        self.__camera_ondemand_controller = ClientOnDemandComponent(component_name='camera', change_event=self.__camera_spawn_change_event)

        self.__service_start_computation = rospy.Service("computation/start", Empty, self.__computation_start)
        self.__service_stop_computation = rospy.Service("computation/stop", Empty, self.__computation_stop)

        self.__service_start_networking = rospy.Service("networking/start", Empty, self.__networking_start)
        self.__service_stop_networking = rospy.Service("networking/stop", Empty, self.__networking_stop)

    def __computation_start(self, msg: EmptyRequest):
        print("Starting computation!")
        self.__is_computing = True
        self.__computation_thread = threading.Thread(target=self.__computation_worker_thread)
        self.__computation_thread.start()
        return EmptyResponse()

    def __computation_stop(self, msg: EmptyRequest):
        self.__is_computing = False
        self.__computation_thread.join()
        print("Computation stopped!")
        return EmptyResponse()

    def __computation_worker_thread(self):
        while self.__is_computing:
            nth = self.n1 + self.n2
            self.n1 = self.n2
            self.n2 = nth

    def __networking_start(self, msg: EmptyRequest):
        print("Starting networking!")
        self.__is_networking = True
        self.__networking_thread = threading.Thread(target=self.__networking_worker_thread)
        self.__networking_thread.start()
        return EmptyResponse()
    
    def __networking_stop(self, msg: EmptyRequest):
        self.__is_networking = False
        self.__networking_thread.join()
        print("Networking stopped!")
        return EmptyResponse()

    def __networking_worker_thread(self):
        while self.__is_networking:
            requests.get(url="http://s2group.cs.vu.nl/")

    def __camera_spawn_change_event(self):
        if not self.__camera_ondemand_controller.is_spawned(): # When despawned in controller, despawn here.
            print("Despawning CameraSensor!")
            self.__camera_process.terminate()
        else:
            print("Spawning CameraSensor!")
            path = self.__dir_path + '/camera/CameraController.py'
            self.__camera_process = sp.Popen(['python3', path])

    def exit(self):
        self.__camera_process.terminate()