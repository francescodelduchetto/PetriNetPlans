import rospy
from abc import ABCMeta, abstractproperty
from AbstractCondition import AbstractCondition

class AbstractServiceCondition(AbstractCondition):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(AbstractServiceCondition, self).__init__()
        # create service proxy
        self.service_proxy = rospy.ServiceProxy(self._service_name, self._service_type)

    # TODO: is this useful for service conditions?
    def get_value(self):
        return None

    @abstractproperty
    def _service_name(self):
        raise NotImplementedError()

    @abstractproperty
    def _service_type(self):
        raise NotImplementedError()
