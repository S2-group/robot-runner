import sys
import signal
from pathlib import Path
from RemoteRunner.Procedures.OutputProcedure import OutputProcedure as output
from RemoteRunner.Controllers.RobotRunnerController import RobotRunnerController as RobotRunner


###     =========================================================
###     |                                                       |
###     |                          Main                         |
###     |       - Ensuring correct usage                        |
###     |       - SIGINT handling (graceful exit procedures)    |
###     |       - Parsing config_path_arg (str) to Path         |
###     |       - Init RobotRunnerController                    |
###     |       - Start RobotRunnerController experiment        |
###     |                                                       |
###     |       * Do not add implementation                     |
###     |         specific overhead here                        |
###     |         Only execution overhead (system)              |
###     |                                                       |
###     =========================================================
def main(config_path_arg):
    # CTRL+C (SIGINT), add necessary graceful exit procedures here.
    def signal_handler(sig, frame):
        output.console_log_bold("SIGINT, Terminating Robot Runner\n", empty_line=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    config_path = Path(config_path_arg)
    output.console_log(f"Initialising experiment for {config_path.absolute()}")

    robot_runner = RobotRunner(config_path)
    robot_runner.do_experiment()


if __name__ == "__main__":
    argv_count = len(sys.argv)
    if argv_count == 2 and sys.argv[1] == "--help":  # Help CLI
        output.console_log("usage: python3 %s [PATH_TO_CONFIG.JSON]" % __file__)
        sys.exit(0)
    elif argv_count == 2:
        main(sys.argv[1]) # Pass config_path_arg to main TODO: Validate that it is a path in ConfigModel or ConfigValidator
    else:
        output.console_log("Incorrect usage, please run with --help to view possible arguments")
        sys.exit(0)