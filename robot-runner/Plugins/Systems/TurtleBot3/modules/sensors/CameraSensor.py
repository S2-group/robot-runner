import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from ExperimentOrchestrator.Architecture.Singleton import Singleton


class CameraSensor(metaclass=Singleton):
    def __init__(self):
        self.node = rclpy.create_node('camera_sensor')
        self.__spawn_service_proxy = self.node.create_client(Empty, '/camera/spawn')
        self.__despawn_service_proxy = self.node.create_client(Empty, '/camera/despawn')
        self.__start_service_proxy = self.node.create_client(Empty, '/camera/start')
        self.__stop_service_proxy = self.node.create_client(Empty, '/camera/stop')

    def spawn(self) -> None:
        self.__call_service(self.__spawn_service_proxy)

    def despawn(self) -> None:
        self.__call_service(self.__despawn_service_proxy)

    def start_recording(self) -> None:
        self.__call_service(self.__start_service_proxy)

    def stop_recording(self):
        self.__call_service(self.__stop_service_proxy)

    def __call_service(self, service_proxy):
        while not service_proxy.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Waiting for service...")

        request = Empty.Request()
        future = service_proxy.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

