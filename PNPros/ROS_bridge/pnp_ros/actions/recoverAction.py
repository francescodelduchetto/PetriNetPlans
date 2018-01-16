import os
import rospy
import Tkinter as tk
import ttk

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from pnp_msgs.msg import ActionFailure
from AbstractAction import AbstractAction

class recoverAction(AbstractAction):

    def _start_action(self):

        # stop the robot
        cancel_goal = GoalID()
        cancel_goal.id = ""
        pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        pub.publish(cancel_goal)
        pub = rospy.Publisher("cmd_vel", Twist, latch=True, queue_size=10)
        twist = Twist()
        twist.linear.x = 0.
        twist.angular.z = 0.
        pub.publish(twist)

        start_sp = rospy.ServiceProxy("start_recovery_execution", Empty)
        start_sp()
        self.params[len(self.params):] = [rospy.Time.now().to_sec()]
        self.params[len(self.params):] = ["recovering"]

    def _stop_action(self):
        if self.params[-1] == "recovering":
            stop_sp = rospy.ServiceProxy("stop_recovery_execution", Empty)
            stop_sp()

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 1:
            elapsed_time = rospy.Time.now().to_sec() - params[-2]
            if params[-1] == "recovering" and elapsed_time > 3:
                return True

        return False
