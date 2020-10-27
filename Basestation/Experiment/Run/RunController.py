import time
import multiprocessing
from multiprocessing import Event

from Basestation.Experiment.Run.IRunController import IRunController
from Common.Procedures.OutputProcedure import OutputProcedure as output

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

    def do_run(self):
        # -- Start run
        output.console_log_WARNING("Calling start_run config hook")
        self.config.start_run(self.run_context)

        # -- Start measurement
        output.console_log_WARNING("... Starting measurement ...")
        self.config.start_measurement(self.run_context)

        # -- Start interaction
        output.console_log_WARNING("Calling interaction config hook")
        during_run = multiprocessing.Process(
            target=self.config.during_run, 
            args=[self.run_context, self.run_completed_event]
        )
        during_run.start()

        # -- Wait for run_duration or until signalled to end
        if self.config.run_duration_in_ms > 0:
            start_time = time.time()
            while not self.run_completed_event.is_set() and (int(round((time.time() - start_time) * 1000)) <= self.config.run_duration_in_ms):
                time.sleep(0.1)
        else:
            self.run_completed_event.wait()

        output.console_log_WARNING("... Run completed ...")

        # -- Stop measurement
        output.console_log_WARNING("... Stopping measurement ...")
        self.config.stop_measurement(self.run_context)

        updated_run_data = self.config.get_updated_run_data(self.run_context)
        if updated_run_data is None:
            row = self.run_context.run_variation
            row['__done'] = 1
            self.data_manager.update_row_data(row)
        else:
            updated_run_data['__done'] = 1
            self.data_manager.update_row_data(updated_run_data)

        # -- Stop run
        output.console_log_WARNING("Calling stop_run config hook")
        self.config.stop_run(self.run_context)