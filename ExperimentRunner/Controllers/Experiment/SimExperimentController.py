from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output
from ExperimentRunner.Controllers.Experiment.IExperimentController import IExperimentController


class SimExperimentController(IExperimentController):
    def do_experiment(self):
        # foreach replication in config, do a run of launch_file
        self.do_run()

    def do_run(self):
        output.console_log("Performing run...")
        self.ros.roslaunch_launch_file(launch_file=self.config.launch_file_path)