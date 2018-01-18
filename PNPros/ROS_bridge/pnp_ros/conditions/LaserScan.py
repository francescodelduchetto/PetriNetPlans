from AbstractTopicCondition import AbstractTopicCondition
import sensor_msgs

class LaserScan(AbstractTopicCondition):

    _topic_name = "/scan"

    _topic_type = sensor_msgs.msg.LaserScan

    def _get_value_from_data(self, data):

<<<<<<< HEAD
	#print len(data.ranges)
=======
        print len(data.ranges)

>>>>>>> 76ce5e800993f965906e69bce6a72a447eef0239
        return str(data.ranges)

    def evaluate(self, params):

        return params == self.last_value
