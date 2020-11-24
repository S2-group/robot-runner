from typing import Callable, List, Tuple
from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents

class EventSubscriptionController:
    __call_back_register: dict = dict()

    @staticmethod
    def subscribe_to_single_event(event: RobotRunnerEvents, callback_method: Callable):
            EventSubscriptionController.__call_back_register[event] = callback_method

    @staticmethod
    def subscribe_to_multiple_events(subscriptions: List[Tuple[RobotRunnerEvents, Callable]]):
        for sub in subscriptions:
            event, callback = sub[0], sub[1]
            EventSubscriptionController.subscribe_to_single_event(event, callback)

    @staticmethod
    def raise_event(event: RobotRunnerEvents, robot_runner_context = None):
        try:
            event_callback = EventSubscriptionController.__call_back_register[event]
        except KeyError:
            return None

        if robot_runner_context:
            return event_callback(robot_runner_context)
        else:
            return event_callback()

    @staticmethod
    def get_event_callback(event: RobotRunnerEvents):
        try:
            return EventSubscriptionController.__call_back_register[event]
        except KeyError:
            return None
