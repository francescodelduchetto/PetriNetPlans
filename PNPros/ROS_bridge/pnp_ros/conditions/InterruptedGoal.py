import rospy
from AbstractTopicCondition import AbstractTopicCondition
from pnp_msgs.msg import PNPGoal

class InterruptedGoal(AbstractTopicCondition):

    _topic_name = "/interrupted_goal"

    _topic_type = PNPGoal

    def _get_value_from_data(self, data):
        return '_'.join([data.name, data.params])

    def evaluate(self, params):
        node = str(params[0])

        if self.last_data:
            current_goal = self.last_value

            return node == current_goal
        else:
            rospy.logwarn("InterruptedGoal condition value not yet initialized")
            return True
