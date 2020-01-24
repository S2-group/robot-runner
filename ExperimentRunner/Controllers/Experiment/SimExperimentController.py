import rospy
from std_msgs.msg import Bool
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output
from ExperimentRunner.Controllers.Experiment.IExperimentController import IExperimentController


class SimExperimentController(IExperimentController):
    run_completed_topic: str = "/robot_runner/run_completed"

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
        self.ros.rosinit_robot_runner_node()
        self.wait_for_simulation()

        # Set programmatic or timed run stop based on config
        # 0  = Programmatic run stop (indefinite run_time)
        # >0 = Timed run stop, stop run after duration
        if self.config.duration == 0:
            self.programmatic_run_stop()
        else:
            self.timed_run_stop()

        self.run_wait_completed()

    def run_completed(self):
        output.console_log("Run completed! Shutting down ROS instances...")
        self.ros.ros_shutdown()

    def timed_run_stop(self):
        pass
        # TODO: set timer and after timer initiate run_completed

    def programmatic_run_stop(self):
        self.ros.subscribe_to_topic(self.run_completed_topic, Bool, self.run_completed)
        output.console_log(f"\033[1mPublish True to {self.run_completed_topic} to programmatically complete run! \033[0m")
