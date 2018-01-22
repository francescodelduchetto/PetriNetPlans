from AbstractTopicCondition import AbstractTopicCondition
from geometry_msgs.msg import Twist

from ast import literal_eval as make_tuple

class Twist(AbstractTopicCondition):

    _topic_name = "/cmd_vel"

    _topic_type = Twist

    def _get_value_from_data(self, data):
        return str((
                data.linear.x,
                data.angular.z
            ))

    def evaluate(self, params):
        try:
            t1 = make_tuple(self.last_value)
            t2 = make_tuple(str(params))
        except ValueError:
            return False
        else:
            tb = (abs(e1 - e2) < 0.01 for e1, e2 in zip(t1, t2))
            return all(tb)
