from AbstractTopicCondition import AbstractTopicCondition
from sensor_msgs.msg import LaserScan

class LaserScan(AbstractTopicCondition):

    _topic_name = "/scan"

    _topic_type = LaserScan

    def _get_value_from_data(self, data):

        return str(data.ranges)

    def evaluate(self, params):

        return params == self.last_value
