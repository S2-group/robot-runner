import os
import sys
import time
import psutil
import subprocess
from std_msgs.msg import Empty

###     =========================================================
###     |                                                       |
###     |                  SignalExperimentEnd                  |
###     |       - Signal the end of experiment to the robot     |
###     |         by using a ROS node. Therefore use either     |
###     |         ROS1 or ROS2, imports fixed accordingly       |
###     |       - Correct usage of ROS1 or ROS2 is guaranteed   |
###     |         by the use of the environment variable        |
###     |                                                       |
###     |       * Any extra functionality needed for            |
###     |         communicating and guaranteeing a graceful     |
###     |         and successful experiment end                 |
###     |         (exit of Remote- and ClientRunner             |
###     |         simultaneously) should be added here          |
###     |                                                       |
###     |       * This file is needed as both rospy and rclpy   |
###     |         only support cleanly spawning one node per    |
###     |         process. When this is done multiple times     |
###     |         from the main robot-runner process, a clean   |
###     |         respawn (spawn and kill) cannot be gauranteed |
###     |                                                       |
###     =========================================================
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

class SignalEndROS1:
    def __init__(self):
        subprocess.Popen('roscore', shell=True)
        time.sleep(5)

        rospy.init_node('signal_experiment_end')

        console_log_bold(msg_ros_node_init)
        pub = rospy.Publisher(ros_topic_sub_url, Empty, queue_size=10)
        console_log_bold(msg_topic_publish)

        rospy.on_shutdown(self.shutdown)

        while True:
            if pub.get_num_connections() > 0:
                pub.publish({})
                break

        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
        sys.exit(0)

    def shutdown(self):
        console_log_bold(msg_run_completed)

if __name__ == "__main__":
    try:
        if ros_version == 1:
            SignalEndROS1()
        else:
            console_log_bold(msg_unsup_ros_ver)
    except KeyboardInterrupt:
        console_log_bold("SIGINT, Terminating SignalExperimentEnd")
