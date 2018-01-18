import os
import rospy
import Tkinter as tk
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from AbstractAction import AbstractAction
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStopStateActionSaver

class recordRecoveryDemonstration(AbstractAction):

    def _start_action(self):
        # stop the robot
        #cancel_goal = GoalID()
        #cancel_goal.id = ""
        #pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        #pub.publish(cancel_goal)
        pub = rospy.Publisher("cmd_vel", Twist, latch=True, queue_size=10)
        twist = Twist()
        twist.linear.x = 0.
        twist.angular.z = 0.
        pub.publish(twist)

        ## Register the recovery trajectory
        # start recording scan and twist
        starting_sp = rospy.ServiceProxy("start_state_action_saver", PNPStartStateActionSaver)
        self.goal_id = rospy.Time.now().to_nsec()
        folder = '%s/workspaces/museum_ws/data/recover_trajectories'  % os.path.expanduser("~")
        filepath = '%s/%s.txt' % (folder, self.goal_id)

        ## create interface to starting demonstration
        window_s = tk.Tk()
        self.start = False
        def start_cb():
            self.start = True
            window_s.destroy()

        label = tk.Label(window_s, text="Press start when you are ready to demonstrate")
        label.pack()
        btn = tk.Button(window_s, text="Start", command = start_cb, height = 1=, width = 20)
        btn.pack()
        window_s.mainloop()

        if self.start:

            # wait until the user moves the robot
            rospy.Subscriber("odom", Odometry, self._twist_callback)
            self._robot_moved = False
            rate = rospy.Rate(20)
            while not self._robot_moved:
                rate.sleep()

            response = starting_sp(str(self.goal_id), filepath, ["LaserScanWindow"], ["Twist"], False).succeeded

            # starting time action
            if response:
                self.params[len(self.params):] = [rospy.Time.now().to_sec()]
                self.params[len(self.params):] = ["recording"]
            else:
                self.params[len(self.params):] = ["done"]
        else:
            self.params[len(self.params):] = ["done"]

    def _stop_action(self):
        if "goal_id" in dir(self):
            stopping_sp = rospy.ServiceProxy("stop_state_action_saver", PNPStopStateActionSaver)
            response = stopping_sp(str(self.goal_id)).succeeded
            # new demostration signal Publisher
            signal_pub = rospy.Publisher("new_recovery_demonstration", String, latch=True, queue_size=10)
            msg = String("")
            signal_pub.publish(msg)

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 0 and params[-1] == "done":
            return True

        if len(params) > 1:
            elapsed_time = rospy.Time.now().to_sec() - params[-2]
            if params[-1] == "recording" and elapsed_time > 3:
                return True

        return False

    def _twist_callback(self, msg):
        if abs(msg.twist.twist.linear.x) > 0.001\
                or abs(msg.twist.twist.angular.z) > 0.001:
            self._robot_moved = True
