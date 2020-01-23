from pathlib import Path
from ExperimentRunner.Controllers.ROSController import ROSController
from ExperimentRunner.Models.ExperimentConfig import ExperimentConfig
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class ExperimentController:
    def __init__(self, config_path: Path):
        self.config = ExperimentConfig(config_path.absolute())
        output.console_log("Experiment config loaded in")
        self.do_experiment()

    def do_experiment(self):
        output.console_log(f"Running in sim: {self.config.sim}, running {self.config.replications} replications")
        # mkdir in output_path -- experiment with guid?
        #while True:
            # For each run:
            # Start the run (ROS Node, Gazebo, roslaunch, etc)
            # mkdir in output_path/experiment_folder for each run OR -> create .csv for each run
        self.do_run()

    def do_run(self):
        output.console_log("Performing run...")
        ROSController().start_fresh_run(launch_file=self.config.launch_file_path)
