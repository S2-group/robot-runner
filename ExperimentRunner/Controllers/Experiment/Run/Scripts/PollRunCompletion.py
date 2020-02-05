import os
import sys
from std_msgs.msg import Bool


def console_log_bold(txt):
    bold_text = f"\033[1m{txt}\033[0m"
    print(f"[ROBOT_RUNNER]:  {bold_text}")


#   |=============================================|
#   |                                             |
#   |                  MESSAGES                   |
#   |                                             |
#   |=============================================|

ros_topic_sub_url = "/robot_runner/run_completed"
msg_ros_node_init = "Initialised ROS Node: poll_run_complete."
msg_topic_publish = "Publish True at /robot_runner/run_completed to complete run!"
msg_tpc_published = "/robot_runner/run_completed topic published true..."
msg_run_completed = "Run_b completed! Shutting down ROS node: robot_runner."
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
    from rosgraph_msgs.msg import Clock


class PollROS1:
    def __init__(self):
        rospy.init_node('poll_run_complete')

        console_log_bold(msg_ros_node_init)
        rospy.Subscriber(ros_topic_sub_url, Bool, self.completed)
        console_log_bold(msg_topic_publish)

        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                r.sleep()
            except ROSInterruptException:
                pass

    def completed(self, data: Bool = True):
        console_log_bold(msg_tpc_published)
        rospy.signal_shutdown('run_completed')

    def shutdown(self):
        console_log_bold(msg_run_completed)


class PollROS2:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node("poll_run_complete")

        console_log_bold(msg_ros_node_init)
        node.create_subscription(Bool, ros_topic_sub_url, self.completed, 10)
        console_log_bold(msg_topic_publish)

        rclpy.spin(node)

    def completed(self, data: Bool = True):
        console_log_bold(msg_tpc_published)
        console_log_bold(msg_run_completed)
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
        console_log_bold("SIGINT, Terminating PollRunCompletion")
