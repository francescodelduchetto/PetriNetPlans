import rospy

from AbstractAction import AbstractAction
from ActionManager import ActionManager
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStopStateActionSaver, PNPConditionValue

class recordDemonstration(AbstractAction):

    def _start_action(self):
        if len(self.params) > 0 and self.params[0] != "":
            # record for the requested action
            goal_action = self.params[0]
            goal_params = self.params[1:]

            goal_str = goal_action + "_" + "_".join(goal_params)
            print "Recording ", goal_str, "(requested)..."
        else:
            # Get the action for which we want a demonstration taking the last interrupted goal
            service_proxy = rospy.ServiceProxy("/PNPConditionValue", PNPConditionValue)

            condition = "InterruptedGoal"
            # get the last interrupted goal
            interrupted_goal = service_proxy(condition).value.split("_")

            goal_action = interrupted_goal[0]
            goal_params = interrupted_goal[1:]

            goal_str = goal_action + "_" + "_".join(goal_params)
            print "Recording ", goal_str, "(interrupted)..."

        # call the starting service
        starting_sp = rospy.ServiceProxy("start_state_action_saver", PNPStartStateActionSaver)
        self._goal_str = goal_str

        response = starting_sp(self._goal_str, "", [], [], True).succeeded

        self._demonstration_filename = None


    def _stop_action(self):
        # call the stopping service
        if self._goal_str:
            stopping_sp = rospy.ServiceProxy("stop_state_action_saver", PNPStopStateActionSaver)
            response = stopping_sp(self._goal_str).succeeded

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 0 and params[0] != "":
            # record for the requested action
            goal_action = params[0]
            goal_params = params[1:]
        else:
            # Get the action for which we want a demonstration taking the last interrupted goal
            service_proxy = rospy.ServiceProxy("/PNPConditionValue", PNPConditionValue)

            condition = "InterruptedGoal"

            # get the last interrupted goal
            interrupted_goal = service_proxy(condition).value.split("_")

            goal_action = interrupted_goal[0]
            goal_params = interrupted_goal[1:]

        if goal_action:
            # check that there is a saver running for this goal
            check_sp = rospy.ServiceProxy("running_state_action_saver", PNPStopStateActionSaver)
            running = check_sp(goal_action + "_".join([""] + goal_params))

            if running.succeeded:
                # Call the ActionManager static method
                return ActionManager.is_goal_reached(goal_action, goal_params)
        return False
