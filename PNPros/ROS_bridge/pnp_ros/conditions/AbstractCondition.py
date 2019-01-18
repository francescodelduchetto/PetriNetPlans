import rospy
from abc import ABCMeta, abstractmethod

class AbstractCondition():
    __metaclass__ = ABCMeta

    def __init__(self):
        self._updates_listeners = []

    @abstractmethod
    def evaluate(self, params):
        raise NotImplementedError()

    @abstractmethod
    def get_value(self):
        raise NotImplementedError()


class ConditionListener():
    __metaclass__ = ABCMeta

    @abstractmethod
    def receive_update(self, condition_name, condition_value):
        raise NotImplementedError()
