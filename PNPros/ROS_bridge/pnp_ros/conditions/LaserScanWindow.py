from AbstractTopicCondition import AbstractTopicCondition
import std_msgs.msg import Float64MultiArray

class LaserScanWindow(AbstractTopicCondition):

    _topic_name = "/scan_history"

    _topic_type = Float64MultiArray

    def _get_value_from_data(self, data):

        print len(data.data)

        return str(data.data)

    def evaluate(self, params):

        return params == self.last_value
