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
    run_started_event = run_completed_event = run_stopped_event = Event()

    def do_run(self):
        # -- Start run
        output.console_log_WARNING("Calling start_run config hook")
        run_start = multiprocessing.Process(
            target=self.config.execute_script_start_run, 
            args=[self.run_context, self.run_started_event]
        )
        run_start.start()
        self.run_started_event.wait()

        # -- Start measurement
        output.console_log_WARNING("... Starting measurement ...")
        self.ros.rosbag_start_recording_topics(
            self.config.topics_to_record,               # Topics to record
            str(self.run_dir.absolute()) + '/topics',   # Path to record .bag to
            f"rosbag_run{self.current_run}"             # Bagname to kill after run
        )
        output.console_log_OK(" + Measurement started successfully!")

        # -- Start interaction
        output.console_log_WARNING("Calling interaction config hook")
        run_interac = multiprocessing.Process(
            target=self.config.execute_script_interaction_during_run, 
            args=[self.run_context, self.run_completed_event]
        )
        run_interac.start()

        # -- Wait for run_duration or until signalled to end
        if self.config.run_duration_in_ms > 0:
            start_time = time.time()
            while (not self.run_completed_event.is_set() and self.config.run_duration_in_ms > 0) \
                and time.time() - start_time <= self.config.run_duration_in_ms:
                time.sleep(0.1)
        else:
            self.run_started_event.wait()

        # -- Stop measurement
        output.console_log_WARNING("... Stopping measurement ...")
        self.ros.rosbag_stop_recording_topics(f"rosbag_run{self.current_run}")
        output.console_log_OK(" + Measurement stopped successfully!")

        # -- Stop run
        output.console_log_WARNING("Calling stop_run config hook")
        run_stop = multiprocessing.Process(
            target=self.config.execute_script_stop_run, 
            args=[self.run_context, self.run_stopped_event]
        )
        run_stop.start()
        self.run_stopped_event.wait()
