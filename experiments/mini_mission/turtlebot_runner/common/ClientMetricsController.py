import rospy
from rospy import Publisher
from rospy import Rate
from std_msgs.msg import Float64
import threading
import psutil
import time

class ClientMetricsController:
    __cpu_pub: Publisher
    __ram_pub: Publisher
    __threads_running: bool

    __cpu_thread: threading.Thread
    __ram_thread: threading.Thread

    __ros_rate: Rate

    def __init__(self):
        self.__cpu_pub = rospy.Publisher('/cpu_usage', Float64, queue_size=10)
        self.__ram_pub = rospy.Publisher('/ram_usage', Float64, queue_size=10)

        self.__ros_rate = rospy.Rate(10)

        self.__cpu_thread = threading.Thread(target=self.__cpu_publisher)
        self.__ram_thread = threading.Thread(target=self.__ram_publisher)

        self.__threads_running = True
        self.__cpu_thread.start()
        self.__ram_thread.start()

    def exit(self):
        print("Metrics stopped!")
        self.__threads_running = False
        time.sleep(1)

    def __cpu_publisher(self):
        while self.__threads_running:
            cpu_usage_percent: float = psutil.cpu_percent(interval=0.0)
            pub_msg: Float64 = Float64()
            pub_msg.data = cpu_usage_percent

            self.__cpu_pub.publish(pub_msg)
            self.__ros_rate.sleep() # 10 Hz | Minimum required sleep for non-blocking CPU_PERCENT calls.
        
        print("\tCPU Thread stopped!")

    def __ram_publisher(self):
        while self.__threads_running:
            ram_usage_percent: float = psutil.virtual_memory().percent
            pub_msg: Float64 = Float64()
            pub_msg.data = ram_usage_percent

            self.__ram_pub.publish(pub_msg)
            self.__ros_rate.sleep() # 10 Hz | Minimum required sleep for non-blocking CPU_PERCENT calls.

        print("\tRAM Thread stopped!")