import rospy
from AbstractAction import AbstractAction
from pnp_msgs.srv import PNPCondition, PNPConditionValue

class doNothing(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Doing nothing for ' + " ".join(self.params) + ' seconds ...')
        #self.starting_time = rospy.Time.now()

    def _stop_action(self):
        rospy.loginfo('Finished doing nothing')

    @classmethod
    def is_goal_reached(cls, params):
        ''' check conditions CurrentGoal and GoalStartingTime '''
        condition_value_sp = rospy.ServiceProxy("/PNPConditionValue", PNPConditionValue)
        condition_eval_sp = rospy.ServiceProxy("/PNPConditionEval", PNPCondition)

        time = 5 #seconds
        if len(params) > 0:
            try:
                time = int(params[0])
            except ValueError:
                return False

        # The goal is reached when we have finished saying what we actually want to say
        current_goal_cond = "CurrentGoal_" + cls.__name__ + "_" + "_".join(params)
        if condition_eval_sp(current_goal_cond).truth_value:
            # check that the elapsed time is enough
            starting_time = condition_value_sp("GoalStartingTime").value
            if starting_time != "None":
                elapsed_time = rospy.Time.now() - rospy.Time.from_sec(float(starting_time))
                if elapsed_time.to_sec() > time:
                    return True

        return False
