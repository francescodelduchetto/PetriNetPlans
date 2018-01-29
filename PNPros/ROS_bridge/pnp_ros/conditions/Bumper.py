from AbstractTopicCondition import AbstractTopicCondition
from std_msgs/Bool

class Bumper(AbstractTopicCondition):

    _topic_name = "/bumper"

    _topic_type = Bool

    def _get_value_from_data(self, data):
        return str(data.data)

    def evaluate(self, _):
        
        return self.last_value == "True"
