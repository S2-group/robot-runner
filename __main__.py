import sys
from pathlib import Path
from ExperimentRunner.Controllers.ExperimentController import ExperimentController
from ExperimentRunner.Utilities.RobotRunnerOutput import RobotRunnerOutput as output


def main(config_path):
    config = Path(config_path)
    output.console_log(f"Initialising experiment for {config.absolute()}")
    ExperimentController(config)


if __name__ == "__main__":
    argv_count = len(sys.argv)
    if argv_count == 2 and sys.argv[1] == "--help":
        output.console_log("usage: python3.7 %s [PATH_TO_CONFIG.JSON]" % __file__)
        sys.exit(0)
    elif argv_count == 2:
        main(sys.argv[1])
    else:
        output.console_log("Incorrect usage, please run with --help to view possible arguments")
