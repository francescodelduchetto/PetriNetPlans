import rospy
from abc import ABCMeta, abstractproperty, abstractmethod
from AbstractCondition import AbstractCondition

class AbstractTopicCondition(AbstractCondition):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(AbstractTopicCondition, self).__init__()
        # subscribe to the topic with a callback
        rospy.Subscriber(self._topic_name, self._topic_type, self._callback)

        # last_data will be None until the subscribed topic will return some data
        self.last_value = None
        self.last_data = None

    def _callback(self, data):
        self.last_data = data
        self.last_value = self._get_value_from_data(data)

        # update all the listeners
        for listener in self._updates_listeners:
            listener.receive_update(self)

    def get_value(self):
        return self.last_value

    def get_data(self):
        return self.last_data

    @abstractmethod
    def _get_value_from_data(self, data):
        raise NotImplementedError()

    @abstractproperty
    def _topic_name(self):
        raise NotImplementedError()

    @abstractproperty
    def _topic_type(self):
        raise NotImplementedError()

    def register_updates_listener(self, obj):
        if issubclass(obj.__class__, ConditionListener):
            self._updates_listeners.append(obj)
        else:
            rospy.logwarn("Object " + str(obj.__class__) + " is not a ConditionListener subclass, cannot be registered as listener")

    def get_name(self):
        return self.__class__.__name__

class ConditionListener():
    __metaclass__ = ABCMeta

    @abstractmethod
    def receive_update(self, condition_name, condition_value):
        raise NotImplementedError()
