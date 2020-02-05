import time
from tabulate import tabulate


class OutputController:
    prev_animation_txt = ""
    robot_runner = "[ROBOT_RUNNER]: "
    animation = "|/-\\"
    idx = 0

    @staticmethod
    def console_log(txt: str, empty_line=False):
        if empty_line:
            print(" " * 100)

        print(f"{OutputController.robot_runner} {txt}")

    @staticmethod
    def console_log_bold(txt: str, empty_line=False):
        bold_text = f"\033[1m{txt}\033[0m"
        OutputController.console_log(bold_text, empty_line)

    @staticmethod
    def console_log_animated(txt: str):
        prev_text = OutputController.prev_animation_txt
        if prev_text != txt and prev_text != "":
            print(" " * 100)

        idx = OutputController.idx
        animation = OutputController.animation
        print(f"{OutputController.robot_runner} {txt} {animation[idx % len(animation)]}", end="\r")
        OutputController.idx += 1
        time.sleep(0.1)

        OutputController.prev_animation_txt = txt

    @staticmethod
    def console_log_tabulate(d: dict):  # Used to output dictionary as readable, pretty table
        headers = ['Key', 'Value']
        data = [(k, v) for k, v in d.items()]
        print(f"\n\n{tabulate(data, headers=headers)}\n\n")
