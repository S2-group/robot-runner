import os
import signal
import psutil
import pathlib
import subprocess
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


class Utils:
    version = 2
    FNULL = open(os.devnull, 'w')  # block output from showing in terminal

    @staticmethod
    def terminate_proc(proc: subprocess.Popen, name):
        output.console_log(f"Terminating process: {name}...")
        proc.send_signal(signal.SIGINT)
        os.kill(proc.pid, signal.SIGINT)
        while proc.poll() is None:
            output.console_log_animated(f"Waiting for graceful exit of process: {name}...")
        output.console_log(f"Successfully terminated process: {name}!", empty_line=True)

    @staticmethod
    def check_process_running(processName: str):
        for proc in psutil.process_iter():
            try:
                # Check if process name contains the given name string.
                if processName.lower() in proc.name().lower():
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return False

    @staticmethod
    def create_dir(absolute_path: str):
        pathlib.Path(absolute_path).mkdir(parents=True, exist_ok=True)

