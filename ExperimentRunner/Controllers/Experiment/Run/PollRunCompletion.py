import sys
import rospy
from rospy import ROSInterruptException
from std_msgs.msg import Bool


class Poll:
    def __init__(self):
        rospy.init_node('robot_runner')
        rospy.loginfo("Initialised ROS Node: robot_runner")
        rospy.Subscriber('/robot_runner/run_completed', Bool, self.completed)
        rospy.loginfo("Publish True at /robot_runner/run_completed to complete run!")
        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                r.sleep()
            except ROSInterruptException:
                rospy.loginfo("Shutdown requested while sleeping, escalating to SIGTERM...")

    def completed(self, data: Bool = True):
        rospy.loginfo("/robot_runner/run_completed topic published true...")
        rospy.signal_shutdown('run_completed')

    def shutdown(self):
        rospy.loginfo("Run completed! Shutting down ROS node: robot_runner")


if __name__ == "__main__":
    Poll()
