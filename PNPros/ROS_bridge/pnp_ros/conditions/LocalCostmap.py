from AbstractTopicCondition import AbstractTopicCondition
from std_msgs.msg import Float64MultiArray

class LocalCostmap(AbstractTopicCondition):

    _topic_name = "/local_costmap"

    _topic_type = Float64MultiArray

    def _get_value_from_data(self, data):

        return str(data.data)

    def evaluate(self, params):

        return params == self.last_value
