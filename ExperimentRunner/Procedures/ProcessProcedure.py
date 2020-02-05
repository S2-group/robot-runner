import os
import sys
import psutil
import signal
import subprocess
from subprocess import Popen
from ExperimentRunner.Controllers.Output.OutputController import OutputController as output


class ProcessProcedure:
    block_out = open(os.devnull, 'w')  # block output from showing in terminal

    @staticmethod
    def process_is_running(process_name: str):
        for proc in psutil.process_iter():
            try:
                # Check if process name contains the given name string.
                if process_name.lower() in proc.name().lower():
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return False

    @staticmethod
    def process_kill_by_name(process_name: str):
        for proc in psutil.process_iter():
            # check whether the process name matches
            if proc.name().lower() == process_name.lower():
                proc.kill()

    @staticmethod
    def process_kill_by_cmdline(contains_cmdline: str):
        for proc in psutil.process_iter():
            # check whether the process name matches
            for line in proc.cmdline():
                if contains_cmdline.lower() in line.lower():
                    proc.kill()

    @staticmethod
    def subprocess_spawn(command: str, name: str):
        try:
            return subprocess.Popen(command, shell=True, stdout=ProcessProcedure.block_out, stderr=subprocess.STDOUT)
        except:
            output.console_log_bold(f"Something went wrong spawning subprocess {name}")
            sys.exit(1)

    @staticmethod
    def subprocess_call(command: str, name: str):
        try:
            subprocess.call(command, shell=True, stdout=ProcessProcedure.block_out, stderr=subprocess.STDOUT)
        except:
            output.console_log_bold(f"Something went wrong calling command: {command} for name: {name}")
            sys.exit(1)

    @staticmethod
    def subprocess_check_output(command:str, name: str):
        try:
            return subprocess.check_output(command, shell=True)
        except:
            output.console_log_bold(f"Something went wrong checking output for command: {command} for name: {name}")
            sys.exit(1)

    @staticmethod
    def subprocess_terminate(proc: Popen, name: str):
        output.console_log(f"Terminating process: {name}...")
        proc.send_signal(signal.SIGINT)
        os.kill(proc.pid, signal.SIGINT)
        while proc.poll() is None:
            output.console_log_animated(f"Waiting for graceful exit of process: {name}...")
        output.console_log(f"Successfully terminated process: {name}!", empty_line=True)