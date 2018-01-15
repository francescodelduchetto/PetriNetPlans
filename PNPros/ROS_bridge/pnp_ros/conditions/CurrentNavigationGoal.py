import rospy
from AbstractTopicCondition import AbstractTopicCondition
from topological_navigation.msg import GotoNodeActionGoal

class CurrentNavigationGoal(AbstractTopicCondition):

    _topic_name = "/topological_navigation/goal"

    _topic_type = GotoNodeActionGoal

    def _get_value_from_data(self, data):
        return data.goal.target

    def evaluate(self, params):
        node = str(params[0])

        if self.last_data:
            current_goal = self.last_value

            return node == current_goal
        else:
            rospy.logwarn("CurrentNavigationGoal condition value not yet initialized")
            return True
