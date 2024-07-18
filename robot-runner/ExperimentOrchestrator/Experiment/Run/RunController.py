import asyncio
from multiprocessing import Process, Queue
import sys
import traceback
from ProgressManager.RunTable.Models.RunProgress import RunProgress
from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ExperimentOrchestrator.Experiment.Run.IRunController import IRunController
from ProgressManager.Output.OutputProcedure import OutputProcedure as output
from EventManager.EventSubscriptionController import RRWebSocketServer


class RunController(IRunController):
    def run_in_subprocess(self, target, *args, **kwargs):
        q = Queue()
        p = Process(target=target, args=(q, *args), kwargs=kwargs)
        p.start()
        result, error = q.get()
        p.join()

        if error is not None:
            ex_type, ex_value, tb_str = error
            message = '%s (in subprocess)\n%s' % (str(ex_value), tb_str)
            raise ex_type(message)

        return result

    def do_run(self):
        def target_process(q, *args, **kwargs):
            self.rr_server = RRWebSocketServer('localhost', 8765) 
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def run_events():
                try:
                    await EventSubscriptionController.raise_remote_event(RobotRunnerEvents.START_RUN, self.rr_server)
                    EventSubscriptionController.raise_event(RobotRunnerEvents.START_RUN, self.run_context)

                    await EventSubscriptionController.raise_remote_event(RobotRunnerEvents.START_MEASUREMENT, self.rr_server)
                    EventSubscriptionController.raise_event(RobotRunnerEvents.START_MEASUREMENT, self.run_context)

                    await EventSubscriptionController.raise_remote_event(RobotRunnerEvents.LAUNCH_MISSION, self.rr_server)
                    EventSubscriptionController.raise_event(RobotRunnerEvents.LAUNCH_MISSION, self.run_context)
                    output.console_log_OK("... Run completed ...")

                    await EventSubscriptionController.raise_remote_event(RobotRunnerEvents.STOP_MEASUREMENT, self.rr_server)
                    EventSubscriptionController.raise_event(RobotRunnerEvents.STOP_MEASUREMENT, self.run_context)

                    await EventSubscriptionController.raise_remote_event(RobotRunnerEvents.POPULATE_RUN_DATA, self.rr_server)
                    updated_run_data = EventSubscriptionController.raise_event(RobotRunnerEvents.POPULATE_RUN_DATA, self.run_context)
                    if updated_run_data is None:
                        row = self.run_context.run_variation
                        row['__done'] = RunProgress.DONE
                        self.data_manager.update_row_data(row)
                    else:
                        updated_run_data['__done'] = RunProgress.DONE
                        self.data_manager.update_row_data(updated_run_data)

                    await EventSubscriptionController.raise_remote_event(RobotRunnerEvents.STOP_RUN, self.rr_server)
                    EventSubscriptionController.raise_event(RobotRunnerEvents.STOP_RUN, self.run_context)

                    q.put((None, None))  # Indicate success
                except Exception:
                    ex_type, ex_value, tb_str = sys.exc_info()
                    q.put((None, (ex_type, ex_value, ''.join(traceback.format_tb(tb_str)))))

            loop.run_until_complete(run_events())
            loop.close()

        self.run_in_subprocess(target_process)
