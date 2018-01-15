import rospy

from AbstractTopicCondition import AbstractTopicCondition
from pnp_msgs.msg import ActionFailure

class FailureSituation(AbstractTopicCondition):

    _topic_name = "/failure_signal"

    _topic_type = ActionFailure

    def _get_value_from_data(self, data):
        return data.cause

    def evaluate(self, params):
        cause = None
        if len(params) > 0:
            cause = str(params[0])

        if self.last_data is not None:
            if (rospy.Time.now().to_sec() - self.last_data.stamp.to_sec()) < 1:
                if cause is not None:
                    return cause == self.last_value
                else:
                    return True

        return False
