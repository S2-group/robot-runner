import time
from std_msgs.msg import Bool
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

    def do_experiment(self):
        current_run: int = 1
        while current_run <= self.config.replications:
            self.do_run(current_run)
            current_run += 1

    def do_run(self, cur_run: int):
        print(f"\n-----------------NEW RUN [{cur_run} / {self.config.replications}]-----------------\n")
        output.console_log("Performing run...")
        self.ros.roslaunch_launch_file(launch_file=self.config.launch_file_path)

        # Init ROS node for Robot Runner and wait for
        # ROS Master and Gazebo Simulator to be running
        self.wait_for_simulation()

        # Set programmatic or timed run stop based on config
        # 0  = Programmatic run stop (indefinite run_time)
        # >0 = Timed run stop, stop run after duration
        if self.config.duration == 0:
            self.programmatic_run_stop()
        else:
            self.timed_run_stop()

        self.run_start()
        self.run_wait_completed()
        self.ros.ros_shutdown()

    def run_completed(self, data: Bool = True):
        self.run_stop()

    def programmatic_run_stop(self):
        self.ros.subscribe_to_topic(self.run_completed_topic, Bool, self.run_completed)
        output.console_log(f"\033[1mPublish True to {self.run_completed_topic} to programmatically complete run! \033[0m")

    def timed_run_stop(self):
        output.console_log(f"Running experiment run for: {self.config.duration}ms == {self.config.duration/1000}s")
        self.timed_stop = True
