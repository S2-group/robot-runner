import os
import sys
from rosgraph_msgs.msg import Clock


def console_log_bold(txt):
    bold_text = f"\033[1m{txt}\033[0m"
    print(f"[ROBOT_RUNNER]:  {bold_text}")


#   |=============================================|
#   |                                             |
#   |                  MESSAGES                   |
#   |                                             |
#   |=============================================|

ros_topic_sub_url = "/clock"
msg_ros_node_init = "Initialised ROS Node: poll_sim_running."
msg_sim_is_cnfrmd = "Simulator is detected to be running correctly."
msg_unsup_ros_ver = "Unsupported $ROS_VERSION environment variable."

ros_version = 0

try:
    ros_version = int(os.environ['ROS_VERSION'])
except ValueError:
    console_log_bold(msg_unsup_ros_ver)
    sys.exit(1)

if ros_version == 1:
    import rospy
    from rospy import ROSInterruptException

if ros_version == 2:
    import rclpy


class PollROS1:
    def __init__(self):
        rospy.init_node("poll_sim_running")
        console_log_bold(msg_ros_node_init)
        rospy.Subscriber(ros_topic_sub_url, Clock, self.clock_callback)
        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                r.sleep()
            except ROSInterruptException:
                pass

    def clock_callback(self, time: Clock):
        if time.clock.sec >= 2:
            rospy.signal_shutdown("sim_running")

    def shutdown(self):
        console_log_bold(msg_sim_is_cnfrmd)


class PollROS2:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node("poll_sim_running")
        console_log_bold(msg_ros_node_init)
        node.create_subscription(Clock, ros_topic_sub_url, self.clock_callback, 10)
        rclpy.spin(node)

    def clock_callback(self, time: Clock):
        if time.clock.sec >= 2:
            console_log_bold(msg_sim_is_cnfrmd)
            sys.exit(0)


if __name__ == "__main__":
    try:
        if ros_version == 1:
            PollROS1()
        elif ros_version == 2:
            PollROS2()
        else:
            console_log_bold(msg_unsup_ros_ver)
    except KeyboardInterrupt:
        console_log_bold("SIGINT, Terminating PollSimRunning")
