import time
import rospy
import signal
import subprocess

from mission.mission import Mission

time.sleep(120)
subprocess.Popen(['roscore'])
time.sleep(5)

rospy.init_node("turtlebot3_custom")

mission = Mission('computation')

def handler(signum, frame):
    print('Ctrl+Z pressed')
    exit()

signal.signal(signal.SIGTSTP, handler)

print("Initializing bringup")
# roslaunch turtlebot3_bringup turtlebot3_robot.launch
subprocess.Popen(['roslaunch', 'turtlebot3_bringup', 'turtlebot3_robot.launch'])

print("Initializing node!")

time.sleep(10)
mission.perform_mission()

while not rospy.is_shutdown():
    rospy.spin()