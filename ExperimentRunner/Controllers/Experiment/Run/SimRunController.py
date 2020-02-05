import time
from pathlib import Path
from ExperimentRunner.Controllers.Experiment.Run.IRunController import IRunController
from ExperimentRunner.Controllers.Output.OutputController import OutputController as output


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
            self.config.topics,                         # Topics to record
            str(self.run_dir.absolute()) + '/topics',   # Path to record .bag to
            f"rosbag_run{self.current_run}"             # Bagname to kill after run
        )

        # If the user set a script to be run while running an experiment run, run it.
        self.run_runscript_if_present()

        # Set run stop, timed or programmatic
        self.set_run_stop()
        self.run_start()

        self.run_wait_completed()

        self.ros.rosbag_stop_recording_topics(f"rosbag_run{self.current_run}")
        self.ros.ros_shutdown()