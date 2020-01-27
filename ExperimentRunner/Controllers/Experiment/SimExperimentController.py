import time
from ExperimentRunner.Utilities.Utils import Utils
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output
from ExperimentRunner.Controllers.Experiment.IExperimentController import IExperimentController


class SimExperimentController(IExperimentController):
    def wait_for_simulation(self):
        sim_running = False
        while not sim_running:
            sim_running = Utils.check_process_running("gzclient")
            output.console_log_animated("Waiting for simulation to be running...")
            time.sleep(1)
        output.console_log("Simulation detected to be running, everything is ready for experiment!")
        time.sleep(3)  # Grace period for simulation

    def do_run(self, cur_run: int):
        run_dir = self.config.exp_dir + f"/run{cur_run}"

        print(f"\n-----------------NEW RUN [{cur_run} / {self.config.replications}]-----------------\n")

        output.console_log("Preparing run...")
        Utils.create_dir(run_dir)
        output.console_log(f"Created run directory: {run_dir}")

        self.ros.roslaunch_launch_file(launch_file=self.config.launch_file_path)

        # Wait until Gazebo simulator is running
        self.wait_for_simulation()

        # Simulation running, start recording topics
        self.ros.rosbag_start_recording_topics(self.config.topics, run_dir + '/topics', f"rosbag_run{cur_run}")

        # Set programmatic or timed run stop based on config
        # 0  = Programmatic run stop (indefinite run_time)
        # >0 = Timed run stop, stop run after duration
        if self.config.duration > 0:
            self.timed_run_stop()
        else:
            self.programmatic_run_stop()

        if self.config.run_script_model.path != "" and self.config.run_script_model.path is not None:
            self.run_script()

        output.console_log("Performing run...")
        self.run_start()
        self.run_wait_completed()
        self.ros.rosbag_stop_recording_topics(f"rosbag_run{cur_run}")
        self.ros.ros_shutdown()
