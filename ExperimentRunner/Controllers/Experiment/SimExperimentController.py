import time
import subprocess
from ExperimentRunner.Utilities.Utils import Utils
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output
from ExperimentRunner.Controllers.Experiment.IExperimentController import IExperimentController


class SimExperimentController(IExperimentController):
    def get_gazebo_clock_time(self):
        return int(str(subprocess.check_output("rostopic echo -n 1 /clock", shell=True)).split('secs: ')[1].split('\\n')[0])

    def wait_for_simulation(self):
        sim_running = False
        while not sim_running:
            clock_time = self.get_gazebo_clock_time()
            sim_running = Utils.check_process_running("gzclient") and \
                          Utils.check_process_running("gzserver") and \
                          clock_time >= 2  # Wait until 2 seconds of sim_time have passed
            output.console_log_animated("Waiting for simulation to be running...")

        output.console_log("Simulation detected to be running, everything is ready for experiment!")
        time.sleep(1)  # Grace period

    def do_run(self, cur_run: int, run_dir: str):
        self.ros.roslaunch_launch_file(launch_file=self.config.launch_file_path)
        self.wait_for_simulation()  # Wait until Gazebo simulator is running

        # Simulation running, start recording topics
        self.ros.rosbag_start_recording_topics(self.config.topics, run_dir + '/topics', f"rosbag_run{cur_run}")

        # Run specified run_script by user in config.json if present
        if self.config.run_script_model.path != "" and self.config.run_script_model.path is not None:
            self.run_script()

        # Set run stop, timed or programmatic
        self.set_run_stop()

        output.console_log("Performing run...")
        self.run_start()
        self.run_wait_completed()
        self.ros.rosbag_stop_recording_topics(f"rosbag_run{cur_run}")
        self.ros.ros_shutdown()
