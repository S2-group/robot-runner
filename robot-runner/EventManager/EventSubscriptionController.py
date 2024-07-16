from contextlib import suppress
import inspect
from typing import Callable, List, Tuple
from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from ExperimentOrchestrator.Architecture.Distributed.RRWebSocketServer import RRWebSocketServer
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext

class EventSubscriptionController:
    __call_back_register = dict()
    __remote_call_register = dict()
    __remote_call_clients_register = dict()

    @staticmethod
    def subscribe_to_single_event(event: RobotRunnerEvents, callback_methods: List[Callable]):
        EventSubscriptionController.__call_back_register[event] = callback_methods

    @staticmethod
    def subscribe_to_multiple_events(subscriptions: List[Tuple[RobotRunnerEvents, Callable]]):
        for sub in subscriptions:
            event, callback = sub[0], List[sub[1]]
            EventSubscriptionController.subscribe_to_single_event(event, callback)

    @staticmethod
    def subscribe_to_multiple_events_multiple_callbacks(subscriptions: List[Tuple[RobotRunnerEvents, List[Callable]]]):
        for sub in subscriptions:
            event, callbacks = sub[0], sub[1]
            EventSubscriptionController.subscribe_to_single_event(event, callbacks)
                
    @staticmethod
    def subscribe_to_multiple_events_multiple_remote_calls(subscriptioins: List[Tuple[RobotRunnerEvents, List[Tuple[str, List[str]]]]]):
        for sub in subscriptioins:
            remote_calls = []
            event, remote_call_tuple = sub[0], sub[1]
            for remote_call in remote_call_tuple:
                remote_calls.append(remote_call[0])
                EventSubscriptionController.__remote_call_clients_register[remote_call[0]] = remote_call[1]
            EventSubscriptionController.__remote_call_register[event] = remote_calls
              
    @staticmethod
    def raise_event(event: RobotRunnerEvents, robot_runner_context = None):
        event_callbacks = []
        try:
            event_callbacks = EventSubscriptionController.__call_back_register[event]
        except KeyError:
            return None

        for event_callback in event_callbacks:
            if robot_runner_context:
                event_callback(robot_runner_context)
            else:
                event_callback()

    @staticmethod
    async def raise_remote_event(event: RobotRunnerEvents, server: RRWebSocketServer):
        event_remote_calls = []
                
        try:
            event_remote_calls = EventSubscriptionController.__remote_call_register[event]
            print(event_remote_calls)
        except KeyError:
            return None
        
        for remote_call in event_remote_calls:
            remote_call_clients = []
            try:
                remote_call_clients = EventSubscriptionController.__remote_call_clients_register[remote_call]
            except KeyError:
                return None
            await server.remote_call(remote_call, remote_call_clients)

    @staticmethod
    def get_event_callback(event: RobotRunnerEvents):
        try:
            return EventSubscriptionController.__call_back_register[event]
        except KeyError:
            return None