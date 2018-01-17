from AbstractTopicCondition import AbstractTopicCondition
import sensor_msgs

class LaserScan(AbstractTopicCondition):

    _topic_name = "/scan"

    _topic_type = sensor_msgs.msg.LaserScan

    def _get_value_from_data(self, data):

	#print len(data.ranges)
        return str(data.ranges)

    def evaluate(self, params):

        return params == self.last_value
