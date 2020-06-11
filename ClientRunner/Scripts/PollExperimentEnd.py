import os
import sys
from std_msgs.msg import Empty
from multiprocessing import Event

from Procedures.OutputProcedure import OutputProcedure as output

###     =========================================================
###     |                                                       |
###     |                  PollExperimentEnd                    |
###     |       - Poll if the experiment has ended by           |
###     |         subscribing to the experiment end ROS topic:  |
###     |         /robot_runner/experiment_completed            |
###     |       - Correct usage of ROS1 or ROS2 is guaranteed   |
###     |         by the use of the environment variable        |
###     |                                                       |
###     |       * Any extra functionality needed for            |
###     |         communicating and guaranteeing a graceful     |
###     |         and successful experiment end                 |
###     |         should be added here                          |
###     |                                                       |
###     |       * This file is needed as both rospy and rclpy   |
###     |         only support cleanly spawning one node per    |
###     |         process. When this is done multiple times     |
###     |         from the main robot-runner process, a clean   |
###     |         respawn (spawn and kill) cannot be gauranteed |
###     |                                                       |
###     =========================================================

# Experiment ended 
exp_topic_END_URL = "/robot_runner/native_experiment_end"
exp_end_CONFIRMED = "Experiment completed! Shutting down ROS node: poll_experiment_end."
exp_end_PUBLISHED = "Publish {} at " + exp_topic_END_URL + " to end the experiment!"

exp_topic_CONTINUE_URL = "/robot_runner/native_experiment_continue"
exp_continue_CONFIRMED = "Experiment continuing! Shutting down ROS node: poll_experiment_end."
exp_continue_PUBLISHED = "Publish {} at " + exp_topic_CONTINUE_URL + " to continue the experiment!"

exp_end_node_INIT = "Initialised ROS Node: poll_experiment_end."
msg_ros_ver_UNSUP = "Unsupported $ROS_VERSION environment variable."

# ROS Version unsupported.
ros_ver_UNSUPPRTD = "Unsupported $ROS_VERSION environment variable."

ros_version = 0

try:
    ros_version = int(os.environ['ROS_VERSION'])
except ValueError:
    output.console_log_bold(msg_ros_ver_UNSUP)
    sys.exit(1)

# Only import if ROS_VERSION == 1, otherwise runtime error!
if ros_version == 1:
    import rospy
    from rospy import ROSException
    from rospy import ROSInterruptException

# Only import if ROS_VERSION == 2, otherwise runtime error!
if ros_version == 2:
    import rclpy

# ======================
# POLL EXP_END FOR ROS1
# ======================
def poll_exp_end_ROS1(event_end: Event, event_continue: Event):
    def exp_end(data: Empty):
        output.console_log_bold(exp_end_CONFIRMED)
        event_end.set()
        rospy.signal_shutdown("experiment_ended")

    def exp_continue(data: Empty):
        output.console_log_bold(exp_continue_CONFIRMED)
        event_continue.set()
        rospy.signal_shutdown("experiment_continued")

    def shutdown():
        output.console_log_bold("poll_experiment_end shutting down...")
    
    rospy.init_node('poll_experiment_end')

    output.console_log_bold(exp_end_node_INIT)

    rospy.Subscriber(exp_topic_END_URL, Empty, exp_end)
    output.console_log_bold(exp_end_PUBLISHED)

    rospy.Subscriber(exp_topic_CONTINUE_URL, Empty, exp_continue)
    output.console_log_bold(exp_continue_PUBLISHED)

    rospy.on_shutdown(shutdown)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            r.sleep()
        except ROSInterruptException:
            sys.exit(1)
        except ROSException:
            sys.exit(1)

# ======================
# POLL EXP_END FOR ROS2
# ======================
def poll_exp_end_ROS2(event_end: Event, event_continue: Event):
    def exp_end(data: Empty):
        output.console_log_bold(exp_end_CONFIRMED)
        event_end.set()
        sys.exit(0)

    def exp_continue(data: Empty):
        output.console_log_bold(exp_continue_CONFIRMED)
        event_continue.set()
        sys.exit(0)
    
    rclpy.init()
    node = rclpy.create_node("poll_experiment_end")

    output.console_log_bold(exp_end_node_INIT)

    node.create_subscription(Empty, exp_topic_END_URL, exp_end, 10)
    output.console_log_bold(exp_end_PUBLISHED)

    node.create_subscription(Empty, exp_topic_CONTINUE_URL, exp_continue, 10)
    output.console_log_bold(exp_continue_PUBLISHED)

    rclpy.spin(node)