#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import GPy
import rospy
import random
import pickle
import threading
import numpy as np
import Tkinter as tk
import ttk

from std_msgs.msg import Float64MultiArray, String
from pnp_msgs.msg import ActionFailure

current_scan_window = []
current_scan_window_data = None
model = None
model_lock = threading.Lock()

def receive_scan_history(data):
    global current_scan_window, current_scan_window_data
    current_scan_window_data = data
    current_scan_window = data.data[:]

def load_model(data):
    global model, model_lock

    folder = '%s/workspaces/museum_ws/data/GPmodels'  % os.path.expanduser("~")
    # acquire permission to modify the model
    model_lock.acquire()
    try:
        model_params = np.load('%s/detector_model_params.npy' % folder)
        X = np.load('%s/detector_model_X.npy' % folder)
        Y = np.load('%s/detector_model_Y.npy' % folder)
    except IOError:
        pass
    else:
        print "X", X.shape, "Y", Y.shape
        multiLS = False
        LS = 400. #was 120.
        F_DIM = len(X[0])
        kernelExpo = GPy.kern.Linear(input_dim=F_DIM,
                                          #lengthscale=LS,
                                          ARD=multiLS)
        model = GPy.models.GPClassification(X, Y, kernel=kernelExpo, initialize=False)
        model.update_model(False) # do not call the underlying expensive algebra on load
        model.initialize_parameter() # Initialize the parameters (connect the parameters up)
        #model['.*len'] = 10.
        #print model_params.shape, model_params
        model[:] = model_params # Load the parameters
        model.update_model(True) # Call the algebra only once
        print "failureDetector"
        print model
        #model.optimize()
    finally:
        # release the lock
        model_lock.release()


if __name__ == "__main__":
    #global current_scan_window
    #global model

    load_model("")

    # init node
    rospy.init_node("failure_detector")

    # scan history Subscriber
    rospy.Subscriber("scan_history", Float64MultiArray, receive_scan_history)

    # failure model updated
    rospy.Subscriber("failure_model_updated", String, load_model)

    # failure Publisher
    signal_pub = rospy.Publisher("failure_signal", ActionFailure, latch=True, queue_size=10)

    # failure trace Publisher
    trace_pub = rospy.Publisher("failure_trace", Float64MultiArray, latch=True, queue_size=10)

    # GUI for human signaling
    window = tk.Tk()

    def human_signal_failure():
        msg = ActionFailure()
        msg.stamp = rospy.Time.now()
        msg.cause = "human"
        signal_pub.publish(msg)

    label = ttk.Label(window, text="Failure signaller")
    label.pack()
    by = ttk.Button(window, text="Signal", command = human_signal_failure)
    by.pack()


    rate = rospy.Rate(20) #hz

    prev_Yp = 0.
    while not rospy.is_shutdown():
        window.update()
        # Check failure with learned model
        if model is None:
            pass
            #rospy.loginfo('%s/detector_model.gp' % folder + " not found")
        elif len(current_scan_window) > 0:
            failure = False

            ## Make prediction
            Xtest = np.array(current_scan_window); Xtest.shape = (1, len(current_scan_window))

            # acquire the lock on the model
            model_lock.acquire()
            #print "predicting"
            try:
                (Yp, var) = model.predict(Xtest)
            except:
                rospy.logwarn("Errore while predicting")
            else:
                #print Yp[0][0]
                if Yp[0][0] != prev_Yp and Yp > 0.75:
                    prev_Yp = Yp[0][0]

                    failure = True
                    #rospy.logerr("FAILURE AUTO-DETECTED, acc: " + str(Yp) + " var: " + str(var))
            #    elif Yp < 0.5 and random.random() < 0.02:
            #        # Save a negative trajectory with 10% probability
            #        folder = '%s/catkin_ws/data/detect_trajectories'  % os.path.expanduser("~")
            #        filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())

            #        pickle.dump(current_scan_window, open(filename, "wb"))
            #        rospy.loginfo("!!! Saving negative failed trajectory in %s" % filename)

                if failure:
                    msg = ActionFailure()
                    msg.stamp = rospy.Time.now()
                    msg.cause = "autodetected"
                    signal_pub.publish(msg)
                    trace_pub.publish(current_scan_window_data)
                    print " Sent failure_signal"
            finally:
                model_lock.release()

        rate.sleep()


    rospy.spin()
