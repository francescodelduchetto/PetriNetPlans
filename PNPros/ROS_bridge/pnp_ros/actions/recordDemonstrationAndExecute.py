import rospy

import actionlib
from ActionManager import ActionManager
from AbstractAction import AbstractAction
from pnp_msgs.msg import PNPAction, PNPActionGoal

class recordDemonstrationAndExecute(AbstractAction):

    def _start_action(self):
        goal_action = self.params[0]
        goal_params = self.params[1:]

        # COnnecting to the action server
        pub = actionlib.SimpleActionClient("/PNP", PNPAction)
        rospy.loginfo("Connecting to /PNP/goal AS...")
        pub.wait_for_server()
        rospy.loginfo("Connected.")

        # recording goal
        pnp_record_goal = PNPActionGoal()
        pnp_record_goal.goal.name = "recordDemonstration"
        pnp_record_goal.goal.params = "_".join([goal_action] + goal_params)
        pnp_record_goal.goal.function = "start"

        # action to be recorder goal
        pnp_action_goal = PNPActionGoal()
        pnp_action_goal.goal.name = goal_action
        pnp_action_goal.goal.params = "_".join(goal_params)
        pnp_action_goal.goal.function = "start"

        # send recording goal
        pub.send_goal(pnp_record_goal.goal)

        # send action goal
        pub.send_goal(pnp_action_goal.goal)

    def _stop_action(self):
        goal_action = self.params[0]
        goal_params = self.params[1:]

        # COnnecting to the action server
        pub = actionlib.SimpleActionClient("/PNP", PNPAction)
        rospy.loginfo("Connecting to /PNP/goal AS...")
        pub.wait_for_server()
        rospy.loginfo("Connected.")

        # recording goal
        pnp_record_goal = PNPActionGoal()
        pnp_record_goal.goal.name = "recordDemonstration"
        pnp_record_goal.goal.params = "_".join([goal_action] + goal_params)
        pnp_record_goal.goal.function = "end"

        # action to be recorder goal
        pnp_action_goal = PNPActionGoal()
        pnp_action_goal.goal.name = goal_action
        pnp_action_goal.goal.params = "_".join(goal_params)
        pnp_action_goal.goal.function = "end"

        # send recording goal
        pub.send_goal(pnp_record_goal.goal)

        # send action goal
        pub.send_goal(pnp_action_goal.goal)

    @classmethod
    def is_goal_reached(cls, params):
        ''' Check that the record action and the recorded action have reached the goal
            (record action reaches the goal when the recorder action reaches its goal...) '''

        goal_action = params[0]
        goal_params = params[1:]

        # Call the ActionManager static method
        return ActionManager.is_goal_reached(goal_action, goal_params)
