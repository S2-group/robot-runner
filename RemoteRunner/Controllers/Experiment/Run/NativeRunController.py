import os
import sys
import time
import subprocess
from Procedures.ProcessProcedure import ProcessProcedure
from Procedures.OutputProcedure import OutputProcedure as output
from Controllers.Experiment.Run.IRunController import IRunController


###     =========================================================
###     |                                                       |
###     |                  NativeRunController                  |
###     |       - Define how to perform a Native run            |
###     |       - Mostly use predefined, generic functions      |
###     |         as defined in the abstract parent             |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific (Native) should be declared here     |
###     |                                                       |
###     =========================================================
class NativeRunController(IRunController):
    def do_run(self):
        self.ros.roscore_start()

        self.wait_for_necessary_topics_and_nodes()

        output.console_log_bold("All necessary nodes and topics available, everything is ready for experiment!")

        self.ros.rosbag_start_recording_topics(
            self.config.topics_to_record,  # Topics to record
            str(self.run_dir.absolute()) + '/topics',  # Path to record .bag to
            f"rosbag_run{self.current_run}"  # Bagname to kill after run
        )

        # If the user set a script to be run while running an experiment run, run it.
        self.run_runscript_if_present()

        # Set run stop, timed or programmatic
        self.set_run_stop()
        self.run_start()

        self.run_wait_completed()
        
        self.ros.rosbag_stop_recording_topics(f"rosbag_run{self.current_run}")
        self.signal_native_run_end()
        self.ros.native_run_end()

    # Needed before ending run. Ending run on ROS2 is for ending on REMOTE PC (this machine), Signalling end is for native device.
    def signal_native_run_end(self):
        if self.config.ros_version == 2:
            output.console_log_bold("Signalling native_run_completed on topic: /robot_runner/native_run_end...")
                                            
            while self.ros.are_topics_available(['/robot_runner/native_run_end']):
                # TODO: Standardize in ProcessProcedure / RosProcedures?
                ProcessProcedure.subprocess_call('ros2 topic pub --once /robot_runner/native_run_end std_msgs/Empty {}', 
                                                'ros2_signal_native_run_end')
