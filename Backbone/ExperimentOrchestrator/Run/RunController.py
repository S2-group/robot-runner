from Backbone.Progress.Models.RunProgress import RunProgress
from Backbone.Events.Models.RobotRunnerEvents import RobotRunnerEvents
from Backbone.Events.EventSubscriptionController import EventSubscriptionController
from Backbone.Architecture.Processify import processify
import time
import multiprocessing
from multiprocessing import Event

from Backbone.ExperimentOrchestrator.Run.IRunController import IRunController
from Backbone.Procedures.OutputProcedure import OutputProcedure as output

from Backbone.Config.RobotRunnerConfig import RobotRunnerConfig

###     =========================================================
###     |                                                       |
###     |                  RunController                        |
###     |       - Define how to perform a run                   |
###     |       - Mostly use predefined, generic functions      |
###     |         as defined in the abstract parent             |
###     |                                                       |
###     |       * Any function which is implementation          |
###     |         specific should be declared here              |
###     |                                                       |
###     =========================================================
class RunController(IRunController):
    @processify
    def do_run(self):
        # -- Start run
        output.console_log_WARNING("Calling start_run config hook")
        EventSubscriptionController.raise_event(RobotRunnerEvents.START_RUN, self.run_context)

        # -- Start measurement
        output.console_log_WARNING("... Starting measurement ...")
        EventSubscriptionController.raise_event(RobotRunnerEvents.START_MEASUREMENT, self.run_context)

        # -- Start interaction
        output.console_log_WARNING("Calling interaction config hook")

        EventSubscriptionController.raise_event(RobotRunnerEvents.DURING_RUN, self.run_context)
        output.console_log_OK("... Run completed ...")

        # -- Stop measurement
        output.console_log_WARNING("... Stopping measurement ...")
        EventSubscriptionController.raise_event(RobotRunnerEvents.STOP_MEASUREMENT, self.run_context)

        updated_run_data = EventSubscriptionController.raise_event(RobotRunnerEvents.POPULATE_RUN_DATA, self.run_context)
        if updated_run_data is None:
            row = self.run_context.run_variation
            row['__done'] = RunProgress.DONE
            self.data_manager.update_row_data(row)
        else:
            updated_run_data['__done'] = RunProgress.DONE
            self.data_manager.update_row_data(updated_run_data)

        # -- Stop run
        output.console_log_WARNING("Calling stop_run config hook")
        EventSubscriptionController.raise_event(RobotRunnerEvents.STOP_RUN, self.run_context)
