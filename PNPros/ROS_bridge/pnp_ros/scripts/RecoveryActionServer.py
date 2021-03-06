import os
import GPy
import rospy
import threading
import numpy as np
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class RecoveryActionServer():

    def __init__(self):
        self._model_lock = threading.Lock()
        self._running = False
        self._model = None

        self._update_model(None)

        self._start_service_provider = rospy.Service("start_recovery_execution",
                Empty, self._start_recovery_execution)

        self._stop_service_provider = rospy.Service("stop_recovery_execution",
                Empty, self._stop_recovery_execution)

        # scan history Subscriber
        rospy.Subscriber("scan", LaserScan,
                self._execution_callback)

        # signal that a new demonstration arrived
        rospy.Subscriber("new_recovery_demonstration", String, self._update_model)

        # cmd_vel publisher
        self._cmdVelPub = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=1)

    def _start_recovery_execution(self, _):
        #if msg:
        rospy.loginfo("Starting recovery execution")
        self._running = True
        return {}

    def _stop_recovery_execution(self, _):
        #if msg:
        rospy.loginfo("Stopping recovery execution")
        self._running = False
        return {}

    def _update_model(self, _):
        # extract target and label data from trajectories
        folder = '%s/catkin_ws/data/recover_trajectories'  % os.path.expanduser("~")
        Xtrain = []
        Ytrain = []
        for filename in os.listdir(folder):
            filepath = folder + "/" + filename
            if filename.endswith(".txt"):
                for line in open(filepath, "r"):
                    x = []
                    y = []
                    state_conds = line.split("\t")[0].split("  ")
                    for state_cond in state_conds:
                        # convert to list
                        x += list(eval(state_cond.split("_")[1]))
                    action_conds = line.split("\t")[1].split("  ")
                    for action_cond in action_conds:
                        # convert to list
                        y += list(eval(action_cond.split("_")[1]))
                    Xtrain.append(x)
                    Ytrain.append(y)

        if len(Xtrain) > 0:
            multiLS = False
            LS = 400. #was 120.
            F_DIM = len(Xtrain[0])

            kernelExpo = GPy.kern.Exponential(input_dim=F_DIM,
                                              lengthscale=LS,
                                              ARD=multiLS) # tails are not long enough
            kernel = kernelExpo # + GPy.kern.White(F_DIM)
            Xtrain = np.array(Xtrain)
            Ytrain = np.array(Ytrain)
            self._model_lock.acquire()
            self._model = GPy.models.GPRegression(Xtrain, Ytrain, kernel)
            self._model.optimize(max_f_eval = 1000)
            self._model_lock.release()

            print "RecoveryActionServer"
            print "X.shape", Xtrain.shape, "Y.shape", Ytrain.shape
            print self._model


    def _execution_callback(self, msg):
        if self._running:
            if self._model is not None:
                current_scan_window = msg.ranges
                Xtest = np.array(current_scan_window)
                Xtest.shape = (1, len(current_scan_window))

                self._model_lock.acquire()
                (Yp, var) = self._model.predict(Xtest)
                self._model_lock.release()

                ## send predicted cmd_vel
                cmdVel = Twist()
                cmdVel.linear.x = Yp[0][0]
                cmdVel.angular.z = Yp[0][1]
                self._cmdVelPub.publish(cmdVel)
                print "predicted", Yp, "with variance", var
            else:
                rospy.logwarn("The recovery model has not been generated")
