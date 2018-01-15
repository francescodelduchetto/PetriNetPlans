import rospy
from AbstractTopicCondition import AbstractTopicCondition
from pnp_msgs.msg import PNPActionGoal

class CurrentGoal(AbstractTopicCondition):

    _topic_name = "/PNP/goal"

    _topic_type = PNPActionGoal

    def _get_value_from_data(self, data):
        return '_'.join([data.goal.name, data.goal.params])

    def evaluate(self, params):
        node = '_'.join(params)

        if self.last_value:
            current_goal = self.last_value

            return node == current_goal
        else:
            #rospy.logwarn("CurrentGoal condition value not yet initialized")
            return False
