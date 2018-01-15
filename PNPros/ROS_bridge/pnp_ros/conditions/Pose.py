from AbstractTopicCondition import AbstractTopicCondition
from nav_msgs.msg import Odometry

from ast import literal_eval as make_tuple

class Pose(AbstractTopicCondition):

    _topic_name = "/odom"

    _topic_type = Odometry

    def _get_value_from_data(self, data):
        return str((
                data.pose.pose.position.x,
                data.pose.pose.position.y,
                data.pose.pose.position.z,
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z
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
