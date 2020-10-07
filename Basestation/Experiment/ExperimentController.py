import os
import sys
import time
import subprocess
from Common.Config.BasestationConfig import BasestationConfig

from Common.Procedures.ProcessProcedure import ProcessProcedure
from Basestation.ROS.IROSController import IROSController
from Basestation.ROS.ROS1Controller import ROS1Controller
from Basestation.ROS.ROS2Controller import ROS2Controller
from Common.Procedures.OutputProcedure import OutputProcedure as output
from Basestation.Experiment.Run.SimRunController import SimRunContoller
from Basestation.Experiment.Run.NativeRunController import NativeRunController

###     =========================================================
###     |                                                       |
###     |                  ExperimentController                 |
###     |       - Init and perform runs of correct type         |
###     |       - Perform experiment overhead                   |
###     |       - Perform run overhead (time_btwn_runs)         |
###     |       - Signal experiment end to robot (ClientRunner) |
###     |                                                       |
###     |       * Experiment config that should be used         |
###     |         throughout the program is declared here       |
###     |         and should not be redeclared (only passed)    |
###     |                                                       |
###     =========================================================
class ExperimentController:
    config: BasestationConfig = None
    ros: IROSController

    def __init__(self, config: BasestationConfig):
        self.config = config
        self.ros = ROS1Controller() if self.config.required_ros_version == 1 else ROS2Controller()

    def do_experiment(self):
        self.config.experiment_path.mkdir(parents=True, exist_ok=True)

        current_run: int = 1
        while current_run <= self.config.number_of_runs:
            run_controller = None
            if self.config.use_simulator:
                run_controller = SimRunContoller(self.config, current_run, self.ros)
            else:
                run_controller = NativeRunController(self.config, current_run, self.ros)

            run_controller.do_run()

            current_run += 1

            time_btwn_runs = self.config.time_between_runs_in_ms
            if time_btwn_runs > 0 and time_btwn_runs is not None:
                output.console_log_bold(f"Run fully ended, waiting for: {time_btwn_runs}ms == {time_btwn_runs / 1000}s")
                time.sleep(time_btwn_runs / 1000)
            else:
                self.signal_experiment_continue()

        if not self.config.use_simulator:
            self.signal_experiment_end()

    def signal_experiment_continue(self):
        if self.config.required_ros_version == 2:
            self.signal_experiment_continue_ros2()

    def signal_experiment_end(self):
        output.console_log_bold("Signalling native_experiment_completed on topic: /robot_runner/native_experiment_completed...")
        
        if self.config.ros_version == 1:
            self.signal_experiment_end_ros1()
        else:
            self.signal_experiment_end_ros2()

        output.console_log_bold("Successfully ended experiment!")

    def signal_experiment_end_ros1(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        exp_end_proc = subprocess.Popen(f"{sys.executable} {dir_path}/Scripts/SignalExperimentEnd.py", shell=True)

        while exp_end_proc.poll() is None:
            output.console_log_animated("Waiting for robot to confirm experiment end...")

        if self.config.ros_version == 1:
            ProcessProcedure.process_kill_by_name('rosmaster')
            ProcessProcedure.process_kill_by_name('roscore')
            ProcessProcedure.process_kill_by_name('rosout')

    def signal_experiment_end_ros2(self):                             
        while self.ros.are_topics_available(['/robot_runner/native_experiment_end']):
            # TODO: Standardize in ProcessProcedure / RosProcedures?
            ProcessProcedure.subprocess_call('ros2 topic pub --once /robot_runner/native_experiment_end std_msgs/Empty {}', 
                                            'ros2_signal_native_experiment_end')

    def signal_experiment_continue_ros2(self):                             
        while self.ros.are_topics_available(['/robot_runner/native_experiment_continue']):
            # TODO: Standardize in ProcessProcedure / RosProcedures?
            ProcessProcedure.subprocess_call('ros2 topic pub --once /robot_runner/native_experiment_continue std_msgs/Empty {}', 
                                            'ros2_signal_native_experiment_continue')