import sys
import signal
from pathlib import Path
from ExperimentRunner.Controllers.Output.OutputController import OutputController as output
from ExperimentRunner.Controllers.RobotRunnerController import RobotRunnerController as RobotRunner


def main(config_path):
    def signal_handler(sig, frame):
        output.console_log_bold("SIGINT, Terminating Robot Runner\n", empty_line=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    config = Path(config_path)
    output.console_log(f"Initialising experiment for {config.absolute()}")
    robot_runner = RobotRunner(config)
    robot_runner.do_experiment()


if __name__ == "__main__":
    argv_count = len(sys.argv)
    if argv_count == 2 and sys.argv[1] == "--help":  # Help CLI
        output.console_log("usage: python3.7 %s [PATH_TO_CONFIG.JSON]" % __file__)
        sys.exit(0)
    elif argv_count == 2:  # Correct usage, continue to program
        main(sys.argv[1])
    else:  # Incorrect usage, display error and exit
        output.console_log("Incorrect usage, please run with --help to view possible arguments")
        sys.exit(0)


