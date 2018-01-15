
from AbstractTopicCondition import AbstractTopicCondition
from pnp_msgs.msg import PNPActionGoal

class GoalStartingTime(AbstractTopicCondition):

    _topic_name = "/PNP/goal"

    _topic_type = PNPActionGoal

    def _get_value_from_data(self, data):
        return str(data.goal_id.stamp.to_sec())

    def evaluate(self, params):
        time = float(params[0])

        if self.last_data:
            starting_time = float(self.last_value)

            return time == starting_time
        else:
            rospy.logwarn("GoalStartingTime condition value not yet initialized")
            return True
