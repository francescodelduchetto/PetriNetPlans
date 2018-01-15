import rospy
from AbstractServiceCondition import AbstractServiceCondition
from strands_navigation_msgs.srv import EstimateTravelTime
from pnp_msgs.srv import PNPConditionValue

class EstimatedTimeout(AbstractServiceCondition):

    _service_name = "/topological_navigation/travel_time_estimator"

    _service_type = EstimateTravelTime

    _ADDITIVE_CONSTANT_TIMEOUT = rospy.Duration.from_sec(5)

    def evaluate(self, params):
        condition_value_sp = rospy.ServiceProxy("/PNPConditionValue", PNPConditionValue)

        current_node = condition_value_sp("CurrentNode").value
        current_goal = condition_value_sp("CurrentGoal").value
        goal_starting_time = condition_value_sp("GoalStartingTime").value
        current_nav_goal = condition_value_sp("CurrentNavigationGoal").value

        if current_goal.split('_')[0] == "goto" :

            # Return False if we are not in a topological node bcs in this case we don't have any estimate
            if current_nav_goal.lower() != "none" and current_node.lower() != "none" \
                    and current_node.lower() != current_nav_goal.lower():
                target_node = current_nav_goal

                estimate_timeout = self.service_proxy(current_node, target_node).travel_time

                starting_time = rospy.Time.from_sec(float(goal_starting_time))
                now = rospy.Time.now()

                time_passed = now - starting_time

                # check timeout expiration
                if time_passed.to_sec() > estimate_timeout.to_sec() + self._ADDITIVE_CONSTANT_TIMEOUT.to_sec():
                    print current_node, target_node
                    return True
        else:
            rospy.logwarn("The current goal is not a navigation goal, not timeout estimate")

        return False
