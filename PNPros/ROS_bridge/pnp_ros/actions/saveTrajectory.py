import os
import rospy
import pickle
import numpy as np
import GPy
from AbstractAction import AbstractAction
from std_msgs.msg import Float64MultiArray, String

class saveTrajectory(AbstractAction):

    def _start_action(self):

        # Save the new trajectory
        scan_window = None
        folder = '%s/workspaces/museum_ws/data/detect_trajectories'  % os.path.expanduser("~")
        if len(self.params) > 0 and self.params[0] == "neg":
            scan_window = rospy.wait_for_message("scan_history", Float64MultiArray, timeout=2)
            filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())
        elif  len(self.params) > 0 and self.params[0] == "fp":
            # take the trace that led to the false positive failure
            scan_window = rospy.wait_for_message("failure_trace", Float64MultiArray, timeout=2)
            filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())
        else:
            scan_window = rospy.wait_for_message("scan_history", Float64MultiArray, timeout=2)
            filename = '%s/%s.traj' % (folder, rospy.Time.now().to_nsec())

        if scan_window:
            pickle.dump(scan_window.data, open(filename, "wb"))
            rospy.loginfo("Saving failed trajectory in %s" % filename)
        else:
            rospy.logwarn("Scan window not received")

        ## Train the model with the new trajectory
        X = []
        negX = []
        for filename in os.listdir(folder):
            filepath = folder + "/" + filename
            if filename.endswith(".traj"):
                if filename.startswith("neg"):
                    negX.append(pickle.load(open(filepath, "rb")))
                else:
                    X.append(pickle.load(open(filepath, "rb")))
        Y = [1] * len(X) + [0] * len(negX); Y = np.array(Y); Y.shape = (len(Y), 1)
        X += negX; X = np.array(X)
        print "INPUT DATA SIZE: ", X.shape
        print "OUTPUT DATA SIZE: ", Y.shape

        # define model
        multiLS = False
        LS = 400. #was 120.
        F_DIM = len(X[0])
        kernelExpo = GPy.kern.Linear(input_dim=F_DIM,
                                          #lengthscale=LS,
                                          ARD=multiLS)
        model = GPy.models.GPClassification(X, Y, kernel=kernelExpo)
        # Constrain all parameters to be positive
        #model['.*len'] = 10.

        # optimize model
        model.optimize()
        print "saveTrajectory"
        print model

        ## save model
        folder = '%s/workspaces/museum_ws/data/GPmodels'  % os.path.expanduser("~")
        np.save('%s/detector_model_params.npy' % folder, model.param_array)
        np.save('%s/detector_model_X.npy' % folder, X)
        np.save('%s/detector_model_Y.npy' % folder, Y)

        # Signal the model update event
        sign_pub = rospy.Publisher("failure_model_updated", String, latch=True, queue_size=10)
        sign_pub.publish(String(""))

        # Signal that the action is done
        self.params.append("done")

    def _stop_action(self):
        pass

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 0 and params[-1] == "done":
            return True

        return False
