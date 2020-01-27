import rospy


class ReadBattery:

    def __init__(self):
        try:
            rospy.init_node("vu_battreader")

            rospy.Subscriber("/mobile_base/commands/charge_level", SensorState, self.sensor_power_callback)

            rospy.on_shutdown(self.shutdown)
            # rospy.spin() tells the program to not exit until you press ctrl + c
            rospy.spin()
        except:
            rospy.loginfo("GoForward node terminated.")

    def sensor_power_callback(self, data):
        # Implement print battery charge

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
