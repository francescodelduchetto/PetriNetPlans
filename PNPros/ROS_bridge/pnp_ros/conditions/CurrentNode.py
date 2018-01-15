from AbstractTopicCondition import AbstractTopicCondition
from std_msgs.msg import String

class CurrentNode(AbstractTopicCondition):

    _topic_name = "/current_node"

    _topic_type = String

    def _get_value_from_data(self, data):
        return data.data

    def evaluate(self, params):
        node = str(params[0])

        return node == self.last_value
