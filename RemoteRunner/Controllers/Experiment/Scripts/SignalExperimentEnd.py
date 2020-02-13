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

ros_topic_sub_url = "/robot_runner/experiment_completed"
msg_ros_node_init = "Initialised ROS Node: signal_experiment_end."
msg_topic_publish = "Publishing True at /robot_runner/experiment_completed signalling experiment end."
msg_run_completed = "Run completed! Shutting down ROS node: signal_experiment_end."
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


class SignalEndROS1:
    def __init__(self):
        rospy.init_node('signal_experiment_end')

        console_log_bold(msg_ros_node_init)
        pub = rospy.Publisher(ros_topic_sub_url, Bool, queue_size=10)
        console_log_bold(msg_topic_publish)

        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                pub.publish(Bool(True))
                r.sleep()
            except ROSInterruptException:
                pass

    def shutdown(self):
        console_log_bold(msg_run_completed)


class SignalEndROS2:
    pass


if __name__ == "__main__":
    try:
        if ros_version == 1:
            SignalEndROS1()
        elif ros_version == 2:
            SignalEndROS2()
        else:
            console_log_bold(msg_unsup_ros_ver)
    except KeyboardInterrupt:
        console_log_bold("SIGINT, Terminating SignalExperimentEnd")
