import sys
import signal
from pathlib import Path
from CLIRegister.CLIRegister import CLIRegister
from CustomErrors.CustomErrors import CommandNotRecognisedError
# from Procedures.ProcessProcedure import ProcessProcedure
# from Procedures.OutputProcedure import OutputProcedure as output
# from Controllers.RobotRunnerController import RobotRunnerController as RobotRunner


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
# def main(config_path_arg, verbose=False):
#     # CTRL+C (SIGINT), add necessary graceful exit procedures here.
#     def signal_handler(sig, frame):
#         output.console_log_bold("SIGINT, Terminating Robot Runner\n", empty_line=True)
#         sys.exit(0)

#     signal.signal(signal.SIGINT, signal_handler)

#     config_path = Path(config_path_arg)
#     output.console_log(f"Initialising experiment for {config_path.absolute()}")

#     ProcessProcedure.verbose = verbose        # Set verbose, if True, subprocesses will output to terminal.
#     robot_runner = RobotRunner(config_path)
#     robot_runner.do_experiment()


