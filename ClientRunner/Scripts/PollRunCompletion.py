import os
import sys
from std_msgs.msg import Bool
from std_msgs.msg import Empty

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../RemoteRunner/')
from Procedures.OutputProcedure import OutputProcedure as output

#   |=============================================|
#   |                                             |
#   |                  MESSAGES                   |
#   |                                             |
#   |=============================================|

ros_topic_sub_url = "/robot_runner/native_run_end"
msg_ros_node_init = "Initialised ROS Node: native_run_complete."
msg_topic_publish = "Publish {} at " + ros_topic_sub_url + " to complete run!"
msg_run_completed = "Native run completed!"
msg_unsup_ros_ver = "Unsupported $ROS_VERSION environment variable."

ros_version = 0

try:
    ros_version = int(os.environ['ROS_VERSION'])
except ValueError:
    output.console_log_bold(msg_unsup_ros_ver)
    sys.exit(1)

if ros_version == 1:
    import rospy
    from rospy import ROSInterruptException

if ros_version == 2:
    import rclpy

class PollROS1:
    def __init__(self):
        rospy.init_node('native_run_complete')

        output.console_log_bold(msg_ros_node_init)
        rospy.Subscriber(ros_topic_sub_url, Bool, self.completed)
        output.console_log_bold(msg_topic_publish)

        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                r.sleep()
            except ROSInterruptException:
                pass

    def completed(self, data: Bool = True):
        rospy.signal_shutdown('run_completed')

    def shutdown(self):
        output.console_log_bold(msg_run_completed)


class PollROS2:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node("native_run_complete")

        output.console_log_bold(msg_ros_node_init)
        node.create_subscription(Empty, ros_topic_sub_url, self.completed, 10)
        output.console_log_bold(msg_topic_publish)

        rclpy.spin(node)

    def completed(self, data: Empty):
        output.console_log_bold(msg_run_completed)
        sys.exit(0)


if __name__ == "__main__":
    try:
        if ros_version == 1:
            PollROS1()
        elif ros_version == 2:
            PollROS2()
        else:
            output.console_log_bold(msg_unsup_ros_ver)
    except KeyboardInterrupt:
        output.console_log_bold("SIGINT, Terminating PollRunCompletion")
