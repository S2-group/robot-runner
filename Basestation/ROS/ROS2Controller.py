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
    processes_sim = ["gzserver", "gzclient"]
    processes_native = ["ros2", "_ros2_daemon"]
    processes_all = processes_sim + processes_native

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
        output.console_log_bold(f"Rosbag2 recording to: {file_path}")
        output.console_log_bold("Recording topics: ")
        # Build 'ros2 bag record --output [file_path] [topic1 topic2 ...]
        command = f" ros2 bag record --output {file_path}"
        for topic in topics:
            command += f" {topic}"
            output.console_log_bold(f" * {topic}")

        ProcessProcedure.subprocess_spawn(command, "ros2bag_record")
        time.sleep(1)  # Give rosbag recording some time to initiate

    def sim_shutdown(self):
        output.console_log("Shutting down sim run...")
        ProcessProcedure.subprocess_call("ros2 service call /reset_simulation std_srvs/srv/Empty {}", "ros2_reset_call")

        time.sleep(1) # Grace period for simulation to reset

        for process in self.processes_all:
            ProcessProcedure.process_kill_by_name(process)

        ProcessProcedure.process_kill_by_cmdline("/opt/ros/")

        while ProcessProcedure.processes_are_running(self.processes_all):
            output.console_log_animated("Waiting for graceful exit...")

        time.sleep(1) # Grace period

        output.console_log("Sim run successfully shutdown!")

    def native_run_end(self):
        output.console_log("Shutting down native run...")

        # TODO: Communicate run_end to native

        #ProcessProcedure.processes_kill_by_names(self.processes_native)
        #ProcessProcedure.process_kill_by_cmdline("/opt/ros/")

        # while ProcessProcedure.processes_are_running(self.processes_native):
        #     output.console_log_animated("Waiting for graceful exit...")

        time.sleep(1)

        output.console_log("Native run successfully shutdown!")
        pass