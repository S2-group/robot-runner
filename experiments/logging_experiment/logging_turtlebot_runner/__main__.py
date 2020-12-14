import rospy
import signal
import subprocess

from mission.mission import Mission

rospy.init_node("turtlebot3_custom")

mission = Mission('computation')

def handler(signum, frame):
    print('Ctrl+Z pressed')
    mission.exit()
    exit()

signal.signal(signal.SIGTSTP, handler)

print("Initializing bringup")
# roslaunch turtlebot3_bringup turtlebot3_robot.launch
subprocess.Popen(['roslaunch', 'turtlebot3_bringup', 'turtlebot3_robot.launch'])

print("Initializing node!")

while not rospy.is_shutdown():
    rospy.spin()