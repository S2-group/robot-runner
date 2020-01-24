import time
from tabulate import tabulate

class RobotRunnerOutput:
    prev_animation_txt = ""
    robot_runner = "[ROBOT_RUNNER]: "
    animation = "|/-\\"
    idx = 0

    @staticmethod
    def console_log(txt: str):
        print(f"{RobotRunnerOutput.robot_runner} {txt}")

    @staticmethod
    def console_log_animated(txt: str):
        prev_text = RobotRunnerOutput.prev_animation_txt
        if  prev_text != txt and prev_text != "":
            print(" " * 100)

        idx = RobotRunnerOutput.idx
        animation = RobotRunnerOutput.animation
        print(f"{RobotRunnerOutput.robot_runner} {txt} {animation[idx % len(animation)]}", end="\r")
        RobotRunnerOutput.idx += 1
        time.sleep(0.1)

        RobotRunnerOutput.prev_animation_txt = txt

    @staticmethod
    def console_log_tabulate(d: dict):                      # Used to output dictionary as readable, pretty table
        headers = ['Key', 'Value']
        data = [(k, v) for k, v in d.items()]
        print(f"\n\n{tabulate(data, headers=headers)}\n\n")
