import time
from RemoteRunner.Controllers.Experiment.Run.IRunController import IRunController
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output


###     =========================================================
###     |                                                       |
###     |                  NativeRunController                  |
###     |       - Define how to perform a Sim run               |
###     |       - Mostly use predefined, generic functions      |
###     |         as defined in the abstract parent             |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific (Sim) should be declared here        |
###     |         like wait_for_simulation()                    |
###     |                                                       |
###     =========================================================
class SimRunContoller(IRunController):
    def wait_for_simulation(self):
        while not self.ros.is_gazebo_running():
            output.console_log_animated("Waiting for simulation to be running...")

        output.console_log("Simulation detected to be running, everything is ready for experiment!")
        time.sleep(1)  # Grace period

    def do_run(self):
        self.ros.roslaunch_launch_file(launch_file=self.config.launch_file_path)
        self.wait_for_simulation()  # Wait until Gazebo simulator is running

        output.console_log("Performing run...")

        # Simulation running, start recording topics
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
        self.ros.sim_shutdown()
