from Backbone.Progress.Models.RunProgress import RunProgress
from Backbone.Events.Models.RobotRunnerEvents import RobotRunnerEvents
from Backbone.Events.EventSubscriptionController import EventSubscriptionController
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
    run_started_event   = Event()
    run_completed_event = Event()
    run_stopped_event   = Event()

    def wait_for_duration(self, event_is_active: bool):
        start_time = time.time()

        while (int(round((time.time() - start_time) * 1000)) <= self.config.run_duration_in_ms):
            if event_is_active:
               if self.run_completed_event.is_set():
                   break

            time.sleep(0.1)

    def do_run(self):
        # -- Start run
        output.console_log_WARNING("Calling start_run config hook")
        EventSubscriptionController.raise_event(RobotRunnerEvents.START_RUN, self.run_context)

        # -- Start measurement
        output.console_log_WARNING("... Starting measurement ...")
        EventSubscriptionController.raise_event(RobotRunnerEvents.START_MEASUREMENT, self.run_context)

        # -- Start interaction
        output.console_log_WARNING("Calling interaction config hook")

        during_run_callback = EventSubscriptionController.get_event_callback(RobotRunnerEvents.DURING_RUN)
        if during_run_callback is not None:
            during_run = multiprocessing.Process(
                target=during_run_callback,
                args=[self.run_context, self.run_completed_event]
            )
            during_run.start()

            if self.config.run_duration_in_ms > 0:
                self.wait_for_duration(True)
        else:
            if self.config.run_duration_in_ms > 0:
                self.wait_for_duration(False)

        output.console_log_WARNING("... Run completed ...")

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
