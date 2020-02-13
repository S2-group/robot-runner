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
msg_ros_node_init = "Initialised ROS Node: poll_experiment_end."
msg_run_completed = "Experiment completed! Shutting down ROS node: poll_experiment_end."
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


class PollExperimentEndROS1:
    def __init__(self):
        rospy.init_node('poll_experiment_end')

        console_log_bold(msg_ros_node_init)
        rospy.Subscriber(ros_topic_sub_url, Bool, self.completed)

        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                r.sleep()
            except ROSInterruptException:
                pass

    def completed(self, data: Bool = True):
        rospy.signal_shutdown('experiment_ended')

    def shutdown(self):
        console_log_bold(msg_run_completed)


class PollExperimentEndROS2:
    pass


if __name__ == "__main__":
    try:
        if ros_version == 1:
            PollExperimentEndROS1()
        elif ros_version == 2:
            PollExperimentEndROS2()
        else:
            console_log_bold(msg_unsup_ros_ver)
    except KeyboardInterrupt:
        console_log_bold("SIGINT, Terminating SignalExperimentEnd")
