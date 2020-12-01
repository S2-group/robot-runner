import time
import rospy
import picamera
from rospy import Service
from std_srvs.srv import (Empty, EmptyRequest, EmptyResponse)
import threading

class CameraController:
    # Operational variable
    __camera = None

    # Predefined variables
    __is_recording:         bool = False

    __start_recording_service: Service
    __stop_recording_service: Service

    __recording_thread: threading.Thread
    __stop_thread: bool = False

    def __init__(self):
        print("\t[CAMERA] Spawned!")
        rospy.init_node("camera_node")
        self.__camera = picamera.PiCamera(framerate=60)
        self.__camera.resolution = (1280, 720)

        self.__start_recording_service = rospy.Service('camera/start', Empty, self.__start_recording)
        self.__stop_recording_service = rospy.Service('camera/stop', Empty, self.__stop_recording)

        self.__recording_thread = threading.Thread(target=self.__recording_worker_thread)
        self.__recording_thread.start()

    def __start_recording(self, msg: EmptyRequest) -> EmptyResponse:
        print("\t[CAMERA] Recording STARTED!")
        self.__is_recording = True
        return EmptyResponse()

    def __stop_recording(self, msg: EmptyRequest) -> EmptyResponse:
        print("\t[CAMERA] Recording STOPPED!")
        self.__is_recording = False
        return EmptyResponse()

    def exit(self):
        self.__stop_recording(EmptyRequest())
        time.sleep(1) # Give time to stop

        self.__stop_thread = True
        self.__recording_thread.join() # Wait for thread to stop and join current process

        self.__camera.close() # Close camera object and release resources
        print("\t[CAMERA] Succesfully stopped thread!")

    def __recording_worker_thread(self):
        thread_recording: bool = False

        def thread_start_recording():
            nonlocal thread_recording
            self.__camera.start_recording('/home/pi/VIDEO_TEST.h264')
            thread_recording = True

        def thread_stop_recording():
            nonlocal thread_recording
            self.__camera.stop_recording()
            thread_recording = False

        while True:                             # Keep thread alive
            while self.__is_recording:          # Record if requested through service
                if not thread_recording:        # If not yet recording (first iteration) start recording
                    thread_start_recording()    # Start recording and prevent second call of start recording
                
                time.sleep(0.5)
            
            if thread_recording:                # Broken out of recording loop, if just recorded, stop recording
                thread_stop_recording()         # Stop recording and prevent second call if not recording again

            if self.__stop_thread:
                print("\t\tThread ordered to stop!")
                break

            time.sleep(0.5)

camera = CameraController()

try:
    while not rospy.is_shutdown():
        time.sleep(0.1)
except:
    pass

# Shutting down
print("\t[CAMERA] Shutting down!")
camera.exit()
exit()