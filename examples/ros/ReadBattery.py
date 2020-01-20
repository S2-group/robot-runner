import roslib
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState


class ReadBattery:
    kobuki_base_max_charge = 160

    def __init__(self):
        try:
            rospy.init_node("vu_battreader")

            rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.sensor_power_callback)

            rospy.on_shutdown(self.shutdown)
            # rospy.spin() tells the program to not exit until you press ctrl + c
            rospy.spin()
        except:
            rospy.loginfo("GoForward node terminated.")

    def sensor_power_callback(self, data):
        rospy.loginfo("Kobuki's battery is now: " + str(
            round(float(data.battery) / float(self.kobuki_base_max_charge) * 100)) + "%")
        if int(data.charger) == 0:
            rospy.loginfo("Not charging at docking station")
        else:
            rospy.loginfo("Charging at docking station")

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
