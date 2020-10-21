import os
import signal
import subprocess
from typing import List
from subprocess import Popen

from Common.Misc.BashHeaders import BashHeaders
from Plugins.Recording.BaseRecorderPlugin import BaseRecorderPlugin
from Common.Procedures.OutputProcedure import OutputProcedure as output

class TopicRecorderPlugin(BaseRecorderPlugin):
    record_proc: Popen = None

    def __init__(self) -> None:
        pass

    def start_recording(self, ros_version: int, file_path: str, topics: List[str]) -> None:
        if not self.record_proc is None:
            output.console_log_FAIL("start_recording IGNORED! >> TopicRecorder " + BashHeaders.UNDERLINE + "start_recording " + BashHeaders.ENDC + "called while already recording!")
            return

        file_path += "-ros2" if ros_version == 2 else "-ros1"
        output.console_log(f"Rosbag starts recording...")
        output.console_log_bold(f"Rosbag recording to: {file_path}")
        output.console_log_bold("Recording topics: ")

        cmd = f"ros2 bag record --output {file_path}" if ros_version == 2 else f"rosbag record -O {file_path}"
        for topic in topics:
            cmd += f" {topic}"
            output.console_log_bold(f" * {topic}")

        self.record_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

        # ProcessProcedure.subprocess_spawn(command, "ros2bag_record")

    def stop_recording(self) -> None:
        os.killpg(os.getpgid(self.record_proc.pid), signal.SIGINT)
        self.record_proc = None