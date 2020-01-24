import time
from tabulate import tabulate


class RobotRunnerOutput:
    was_animated = False                # Needed for correct output in terminal after animation
    robot_runner = "[ROBOT_RUNNER]: "
    animation = "|/-\\"
    idx = 0

    @staticmethod
    def console_log(txt: str):
        if RobotRunnerOutput.was_animated:
            print("\n", end="")                             # Clear animated line with a new line so that new output
            RobotRunnerOutput.was_animated = False          # Does not glitch through older output (animated output)

        print(f"{RobotRunnerOutput.robot_runner} {txt}")

    @staticmethod
    def console_log_loading_animated(txt: str):
        idx = RobotRunnerOutput.idx
        animation = RobotRunnerOutput.animation
        print(f"{RobotRunnerOutput.robot_runner} {txt} {animation[idx % len(animation)]}", end="\r")
        RobotRunnerOutput.idx += 1
        time.sleep(0.1)
        RobotRunnerOutput.was_animated = True               # Reset to False after next non-animated output

    @staticmethod
    def console_log_tabulate(d: dict):                      # Used to output dictionary as readable, pretty table
        headers = ['Key', 'Value']
        data = [(k, v) for k, v in d.items()]
        print(f"\n\n{tabulate(data, headers=headers)}\n\n")
