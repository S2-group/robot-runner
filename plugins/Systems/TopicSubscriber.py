from typing import Callable, Dict, List
from rospy import Subscriber

class TopicSubscriber:
    def subscribe_to_topic(self, topic: str, datatype, callback: Callable) -> Subscriber:
        return Subscriber(topic, datatype, callback)

    def subscribe_to_multiple_topics_on_one_callback(self, topic_datatype_map: Dict, callback: Callable) -> List[Subscriber]:
        subscription_handlers = []
        for topic, datatype in topic_datatype_map.items():
            subscription_handlers.append(Subscriber(topic, datatype, callback))

        return subscription_handlers

    def unregister_from_single_subscription(self, subscription: Subscriber):
        subscription.unregister()

    def unregister_from_multiple_subscriptions(self, subscriptions: List[Subscriber]):
        for subscription in subscriptions:
            self.unregister_from_single_subscription(subscription)
    
            