import os
import rospy
import Tkinter as tk

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from pnp_msgs.msg import ActionFailure
from AbstractAction import AbstractAction
from std_msgs.msg import Float64MultiArray

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

        ## Ask for confirmation by human

        # confirmation publisher
        conf_pub = rospy.Publisher("failure_signal_confirmation", ActionFailure, latch=True, queue_size=10)

        # read the failure trace_data
	try:
	        failure_trace = rospy.wait_for_message("failure_trace", Float64MultiArray, timeout=5)
	except Exception as e:
		rospy.logwarn("failure trace not received, stopping recovery")
		self.params[len(self.params):] = ["done"]

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

        label = tk.Label(window, text="Was it actually a dangerous situation?")
        label.pack()
        by = tk.Button(window, text="Yes", command = confirm_y, height=10, width=10)
        bn = tk.Button(window, text="No", command = confirm_n, height=10, width=10)
        bd = tk.Button(window, text="Dunno", command = confirm_d, height=10, width=10)
        by.pack()
        bn.pack()
        bd.pack()
        window.mainloop()

        # NOTE: always execute
        #self.confirmed = True

        print "confirmed: ", self.confirmed

        if self.confirmed == True:
            # send failure confirmation
            msg = ActionFailure()
            msg.stamp = rospy.Time.now()
            msg.trace = failure_trace
            msg.cause = "positive"
            conf_pub.publish(msg)
            # start recovery service
            start_sp = rospy.ServiceProxy("start_recovery_execution", Empty)
            start_sp()
            self.params[len(self.params):] = [rospy.Time.now().to_sec()]
            self.params[len(self.params):] = ["recovering"]
        elif self.confirmed == False:
            # send failure confirmation
            msg = ActionFailure()
            msg.stamp = rospy.Time.now()
            msg.trace = failure_trace
            msg.cause = "falsepositive"
            conf_pub.publish(msg)
            ### NOTE do not makes sense anymore to send this
            #pub = rospy.Publisher("failure_signal", ActionFailure, queue_size=10, latch=True)
            #msg = ActionFailure()
            #msg.stamp = rospy.Time.now()
            #msg.cause = "falsepositive"
            #pub.publish(msg)

            # finished action
            self.params[len(self.params):] = ["done"]
        else:
            # send failure confirmation
            msg = ActionFailure()
            msg.stamp = rospy.Time.now()
            msg.trace = failure_trace
            msg.cause = "dunno"
            conf_pub.publish(msg)
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
            if params[-1] == "recovering" and elapsed_time > 6:
                return True

        return False
