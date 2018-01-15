import rospy
import actionlib

from topological_navigation.msg import GotoNodeActionGoal, GotoNodeAction
from actionlib_msgs.msg import GoalID
from AbstractAction import AbstractAction
from pnp_msgs.srv import PNPCondition
from geometry_msgs.msg import Twist


class goto(AbstractAction):

    def _start_action(self):
        goal_topo = str(self.params[0])

        self.nav_goal = GotoNodeActionGoal()
        self.nav_goal.goal.target = goal_topo

        self.nav_ac = actionlib.SimpleActionClient("/topological_navigation", GotoNodeAction)
        rospy.loginfo("Connecting to /topological_navigation AS...")
        self.nav_ac.wait_for_server()
        rospy.loginfo("Connected.")

        # send navigation goal
        self.nav_ac.send_goal(self.nav_goal.goal)
        rospy.loginfo("Waiting for result...")

        # cmdvel Publisher
        self._cmdVelPub = rospy.Publisher("cmd_vel", Twist, latch=True, queue_size=10)

        print "START ACTION GOTO"


    def _stop_action(self):

        cancel_goal = GoalID()
        cancel_goal.id = self.nav_goal.goal_id.id
        pub = rospy.Publisher('/topological_navigation/cancel', GoalID, queue_size=10)
        pub.publish(cancel_goal)

        # let's be sure that the robot stops
        #cancel_goal.id = ""
        #pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        #pub.publish(cancel_goal)
        cmdVel = Twist()
        cmdVel.linear.x = .0
        cmdVel.angular.z = .0
        self._cmdVelPub.publish(cmdVel)

        print "STOP ACTION GOTO"

    @classmethod
    def is_goal_reached(cls, params):
        '''Check condition CurrentNode'''
        goal_node = str(params[0])

        service_proxy = rospy.ServiceProxy("/PNPConditionEval", PNPCondition)

        condition = "CurrentNode_" + goal_node
        reached = service_proxy(condition).truth_value
        return reached

        #if self.nav_ac.get_result():
        #    # print the result of navigation
        #    nav_res = self.nav_ac.get_result()
        #    rospy.loginfo("Result: " + str(nav_res))
        #    nav_state = self.nav_ac.get_state()
        #    rospy.loginfo("State: " + str(nav_state))
        #    return True
        #else:
        #    return False
