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

        # Ask for confirmation by human
        window = tk.Tk()
        self.confirmed = None

        def confirm_y():
            self.confirmed = True
            window.destroy()

        def confirm_n():
            self.confirmed = False
            window.destroy()

        def confirm_d():
            window.destroy()

        label = ttk.Label(window, text="Was it actually a dangerous situation?")
        label.pack()
        by = ttk.Button(window, text="Yes", command = confirm_y)
        bn = ttk.Button(window, text="No", command = confirm_n)
        bd = ttk.Button(window, text="Dunno", command = confirm_d)
        by.pack()
        bn.pack()
        bd.pack()
        window.mainloop()

        # NOTE: always execute
        #self.confirmed = True

        print "confirmed: ", self.confirmed

        if self.confirmed == True:
            start_sp = rospy.ServiceProxy("start_recovery_execution", Empty)
            start_sp()
            self.params[len(self.params):] = [rospy.Time.now().to_sec()]
            self.params[len(self.params):] = ["recovering"]
        elif self.confirmed == False:
            ## signal wrong failure detection
            pub = rospy.Publisher("failure_signal", ActionFailure, queue_size=10, latch=True)
            msg = ActionFailure()
            msg.stamp = rospy.Time.now()
            msg.cause = "falsepositive"
            pub.publish(msg)

            # finished action
            self.params[len(self.params):] = ["done"]
        else:
            # finished action
            self.params[len(self.params):] = ["done"]

    def _stop_action(self):
        if self.params[-1] == "recovering":
            stop_sp = rospy.ServiceProxy("stop_recovery_execution", Empty)
            stop_sp()

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 0 and params[-1] == "done":
            return True

        if len(params) > 1:
            elapsed_time = rospy.Time.now().to_sec() - params[-2]
            if params[-1] == "recovering" and elapsed_time > 3:
                return True

        return False
