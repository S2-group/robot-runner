import os
import signal
import tempfile
import subprocess
from typing import List
from subprocess import Popen

from Common.Misc.BashHeaders import BashHeaders
from Common.Config.BasestationConfig import BasestationConfig, RobotRunnerContext
from Plugins.Recording.BaseRecorderPlugin import BaseRecorderPlugin
from Common.Procedures.OutputProcedure import OutputProcedure as output

class TopicRecorderPlugin(BaseRecorderPlugin):
    record_proc: Popen             = None
    config:      BasestationConfig = None
    file_path:   str               = None

    def __init__(self, config: BasestationConfig) -> None:
        """Initialize a TopicRecorder"""
        self.config = config
        self.file_path = tempfile.gettempdir() + '/robot_runner/' + self.config.name + '/temp'
    
    def start_recording(self, context: RobotRunnerContext) -> None:
        if not self.record_proc is None:
            output.console_log_FAIL("start_recording IGNORED! >> TopicRecorder " + BashHeaders.UNDERLINE + "start_recording " + BashHeaders.ENDC + "called while already recording!")
            return

        if self.config.output_plugin_raw_data:
            self.file_path = str(context.run_dir.absolute()) + '/TopicRecorder'
            
        output.console_log(f"Rosbag starts recording...")
        output.console_log_bold(f"Rosbag recording to: {self.file_path}")
        output.console_log_bold("Recording topics: ")

        cmd = f"ros2 bag record --output {self.file_path}" if self.config.required_ros_version == 2 else f"rosbag record -O {self.file_path}"
        for topic in self.config.topics_to_record:
            cmd += f" {topic}"
            output.console_log_bold(f" * {topic}")

        self.record_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                            shell=True, preexec_fn=os.setsid)

        # ProcessProcedure.subprocess_spawn(command, "ros2bag_record")

    def stop_recording(self) -> None:
        os.killpg(os.getpgid(self.record_proc.pid), signal.SIGINT)
        self.record_proc = None

        # TODO: return data to user

    def __convert_to_txt_ROS1(self):
        pass

    def __convert_to_txt_ROS2(self):
        subprocess.Popen("ros2 bag play TopicRecorder_0.db3")

        for topic in self.config.topics_to_record:
            subprocess.Popen(f"ros2 topic echo {topic} /topic_data_type > data.txt")