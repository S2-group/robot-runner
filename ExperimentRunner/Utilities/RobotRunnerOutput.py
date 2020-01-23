import time


class RobotRunnerOutput:
    animation = "|/-\\"
    idx = 0

    @staticmethod
    def console_log(str):
        print(f"[ROBOT_RUNNER]: {str}")

    @staticmethod
    def console_log_loading_animated(str):
        idx = RobotRunnerOutput.idx
        animation = RobotRunnerOutput.animation
        print(f"{str} {animation[idx % len(animation)]}", end="\r")
        RobotRunnerOutput.idx += 1
        time.sleep(0.1)
