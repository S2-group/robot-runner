from Plugins.Systems.TurtleBot3.modules.sensors.CPUSensor import CPUSensor
from Plugins.Systems.TurtleBot3.modules.sensors.RAMSensor import RAMSensor
from Plugins.Systems.TurtleBot3.modules.sensors.BatterySensor import BatterySensor
# from Plugins.Systems.TurtleBot3.modules.sensors.OdomSensor import OdomSensor

import threading
import time


class MetricsRecorder:
    __cpu_sensor: CPUSensor
    __ram_sensor: RAMSensor
    __bat_sensor: BatterySensor

    __may_threads_exist: bool
    __is_recording: bool
    __recording_thread: threading.Thread

    __metrics_file = None
    __file_location: str

    def __init__(self, file_location: str):
        self.__cpu_sensor = CPUSensor()
        self.__ram_sensor = RAMSensor()
        self.__bat_sensor = BatterySensor()
        # self.__odom_sensor = OdomSensor()
        self.__file_location = file_location

    def stop_recording(self):
        print("Stop recording metrics")
        self.__is_recording = False
        time.sleep(1)  # grace period
        self.__metrics_file.close()

    def start_recording(self):
        self.__metrics_file = open(self.__file_location, 'w+')
        self.__is_recording = True
        self.__recording_thread = threading.Thread(target=self.__recording)
        self.__recording_thread.start()

    def __recording(self):
        while self.__is_recording:
            cpu_usage: float = self.__cpu_sensor.get_percentage()
            ram_usage: float = self.__ram_sensor.get_percentage()
            bat_percentage: float = self.__bat_sensor.get_percentage()
            # msg_count:      int   = self.__odom_sensor.get_counts()
            # msg_count:      int   = 0
            # timestamp:      float = time.time()

            self.__metrics_file.write(str(cpu_usage) + "," + str(ram_usage) + "," + str(bat_percentage) + "\n")
            # self.__metrics_file.write(timestamp + "," + str(cpu_usage) + "," + str(ram_usage) + "," + str(bat_percentage) + "," + str(msg_count) + "\n")
            time.sleep(0.1)

        print("Stopped recording thread")