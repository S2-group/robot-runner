import os
import sys
import time
import subprocess

from Common.Misc.BashHeaders import BashHeaders
from Common.Config.BasestationConfig import BasestationConfig
from Basestation.Experiment.Run.IRunController import IRunController
from Common.Procedures.OutputProcedure import OutputProcedure as output


###     =========================================================
###     |                                                       |
###     |                  RunController                        |
###     |       - Define how to perform a run                   |
###     |       - Mostly use predefined, generic functions      |
###     |         as defined in the abstract parent             |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific should be declared here              |
###     |                                                       |
###     =========================================================
class RunController(IRunController):
    def do_run(self):
        output.console_log_WARNING("Calling start_run config hook")
        self.config.execute_script_start_run(self.run_context)

        output.console_log_WARNING("... Starting measurement ...")
        # Record topics
        self.ros.rosbag_start_recording_topics(
            self.config.topics_to_record,               # Topics to record
            str(self.run_dir.absolute()) + '/topics',   # Path to record .bag to
            f"rosbag_run{self.current_run}"             # Bagname to kill after run
        )
        output.console_log_OK(" + Measurement started successfully!")

        #if self.config.launch_file_path != "":
        # If the user set a script to be run while running an experiment run, run it.
        #self.run_runscript_if_present()

        output.console_log_WARNING("Calling interaction config hook")
        self.config.execute_script_interaction_during_run(self.run_context)

        # TODO: Run stop should be timed (duration > 0, or programmatic: ROS Service)
        # Set run stop, timed or programmatic
        # self.set_run_stop()
        # self.run_start()

        # Either event based, or timed based -> method returns when done
        self.run_wait_completed()

        output.console_log_WARNING("Calling stop_run config hook")
        self.config.execute_script_stop_run(self.run_context)

        output.console_log_WARNING("... Stopping measurement ...")
        self.ros.rosbag_stop_recording_topics(f"rosbag_run{self.current_run}")
        output.console_log_OK("\t Measurement stopped successfully!")