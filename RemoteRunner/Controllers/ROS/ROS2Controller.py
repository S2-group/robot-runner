import time
from pathlib import Path
from Procedures.ProcessProcedure import ProcessProcedure
from Controllers.ROS.IROSController import IROSController
from Procedures.OutputProcedure import OutputProcedure as output


###     =========================================================
###     |                                                       |
###     |                     ROS2Controller                    |
###     |       - Define communications with ROS2               |
###     |           - ROS2 does not need a roscore process      |
###     |           - Launch a launch file (.launch.py)         |
###     |           - Start ros2 bag recording of topics        |
###     |                - Stop is automatic (for now)          |
###     |           - Define graceful shutdown procedure        |
###     |             for both Native and Sim runs              |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific (ROS2) should be declared here       |
###     |                                                       |
###     =========================================================
class ROS2Controller(IROSController):
    def roscore_start(self):
        pass  # ROS2 does not have / need roscore.

    def rosbag_stop_recording_topics(self, bag_name):
        pass  # TODO: For now OK, needs to terminate ros2bag process in future.

    def roslaunch_launch_file(self, launch_file: Path):
        output.console_log(f"ros2 launch {launch_file}")
        command = f"ros2 launch {launch_file}"
        self.roslaunch_proc = ProcessProcedure.subprocess_spawn(command, "ros2_launch_file")

    def rosbag_start_recording_topics(self, topics, file_path, bag_name):
        file_path += "-ros2"
        output.console_log(f"Rosbag2 starts recording...")
        output.console_log_bold("Recording topics: ")
        # Build 'rosbag record -O filename [/topic1 /topic2 ...] __name:=bag_name' command
        command = f" ros2 bag record --output {file_path}"
        for topic in topics:
            command += f" {topic}"
            output.console_log_bold(f" * {topic}")

        ProcessProcedure.subprocess_spawn(command, "ros2bag_record")
        time.sleep(1)  # Give rosbag recording some time to initiate

    def sim_shutdown(self):
        output.console_log("Shutting down sim run...")
        ProcessProcedure.subprocess_call("ros2 service call /reset_simulation std_srvs/srv/Empty {}", "ros2_reset_call")

        ProcessProcedure.process_kill_by_name("gzserver")
        ProcessProcedure.process_kill_by_name("gzclient")
        ProcessProcedure.process_kill_by_name("_ros2_daemon")
        ProcessProcedure.process_kill_by_name("ros2")
        ProcessProcedure.process_kill_by_cmdline("/opt/ros/")

        while ProcessProcedure.process_is_running("gzserver") or \
                ProcessProcedure.process_is_running("gzclient") or \
                ProcessProcedure.process_is_running("ros2") or \
                ProcessProcedure.process_is_running("_ros2_daemon"):
            output.console_log_animated("Waiting for graceful exit...")

        output.console_log("Sim run successfully shutdown!")

    def native_shutdown(self):
        output.console_log("Shutting down native run...")
        # TODO: Implement this, impossible as of now because of know Linux Kernel bug
        #       on ARM devices. Raspberry Pi overheats and crashes due to inability to throttle CPU.
 
        # Get all nodes, for each node ros2 lifecycle nodename shutdown
        pass
        # ======= ROS 1 =======
        # output.console_log("Shutting down native run...")
        # ProcessProcedure.subprocess_call('rosnode kill -a', "rosnode_kill")
        # ProcessProcedure.process_kill_by_name('rosmaster')
        # ProcessProcedure.process_kill_by_name('roscore')
        # ProcessProcedure.process_kill_by_name('rosout')
        #
        # while ProcessProcedure.process_is_running('rosmaster') and \
        #       ProcessProcedure.process_is_running('roscore') and \
        #       ProcessProcedure.process_is_running('rosout'):
        #     output.console_log_animated("Waiting for roscore to gracefully exit...")
        #
        # output.console_log("Native run successfully shutdown!")