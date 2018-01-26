#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import GPy
import time
import rospy
import random
import pickle
import threading
import numpy as np
import Tkinter as tk

from std_msgs.msg import Float64MultiArray, String
from pnp_msgs.msg import ActionFailure

POSITIVE_THRESHOLD = 0.78

current_scan_window = []
current_scan_window_data = None
model = None
model_lock = threading.Lock()
failure_confirmed = True
last_failure_trace = None
meanX = None
stdX = None

def receive_scan_history(data):
    global current_scan_window, current_scan_window_data
    current_scan_window_data = data
    current_scan_window = data.data[:]

def save_new_trajectory(type="positive", trace_data=None):
    global current_scan_window_data

    if trace_data is None:
        trace_data = current_scan_window_data

    folder = '%s/workspaces/museum_ws/data/detect_trajectories'  % os.path.expanduser("~")
    if type == "negative":
        filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())
    elif type == "positive":
        filename = '%s/%s.traj' % (folder, rospy.Time.now().to_nsec())

    pickle.dump(trace_data.data, open(filename, "wb"))
    rospy.loginfo("Saving failed trajectory in %s" % filename)

def  failure_confirmation(data):
    if data.cause == "falsepositive":
        save_new_trajectory(type="negative", trace_data=data.trace)
        load_model("")

def load_model(_):
    global model, model_lock, meanX, stdX

    folder = '%s/workspaces/museum_ws/data/detect_trajectories'  % os.path.expanduser("~")
    ## Train the model with the new trajectory
    X = []
    Y = []
    for filename in os.listdir(folder):
        filepath = folder + "/" + filename
        if filename.endswith(".traj"):
            if filename.startswith("neg"):
		e = list(pickle.load(open(filepath, "rb")))
                Y.append(0)
            else:
		e = list(pickle.load(open(filepath, "rb")))
		Y.append(1)
            X.append(e)
    Y = np.array(Y); Y.shape = (len(Y), 1)
    X = np.array(X)
    print "INPUT DATA SIZE: ", X.shape
    print "OUTPUT DATA SIZE: ", Y.shape

    meanX = X.mean()
    stdX = X.std()
    X -= meanX
    X /= stdX

    # acquire permission to modify the model
    model_lock.acquire()
    try:
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
    except:
        pass
    else:
        print "failureDetector"
        print model
    finally:
        # release the lock
        model_lock.release()


if __name__ == "__main__":
    time.sleep(4)
    #global current_scan_window
    #global model

    load_model("")

    # init node
    rospy.init_node("failure_detector")

    # scan history Subscriber
    rospy.Subscriber("scan_history", Float64MultiArray, receive_scan_history)

    # failure model updated
    #rospy.Subscriber("failure_model_updated", String, load_model)

    # to hear back from the plan after the failure
    rospy.Subscriber("failure_signal_confirmation", ActionFailure, failure_confirmation)

    # failure Publisher
    signal_pub = rospy.Publisher("failure_signal", ActionFailure, latch=True, queue_size=10)

    ## failure trace Publisher
    trace_pub = rospy.Publisher("failure_trace", Float64MultiArray, latch=True, queue_size=10)

    # GUI for human signaling
    window = tk.Tk()

    def human_signal_failure():
        # save trajectory
        save_new_trajectory(type="positive")
        # publish the failure
        msg = ActionFailure()
        msg.stamp = rospy.Time.now()
        msg.cause = "human"
        signal_pub.publish(msg)
	time.sleep(5)
        # update the model
        load_model("")

    label = tk.Label(window, text="Failure signaller")
    label.pack()
    by = tk.Button(window, text="Signal", command = human_signal_failure, height = 10, width = 20)
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
	    Xtest -= meanX
	    Xtest /= stdX
            try:
                # acquire the lock on the model
                model_lock.acquire()
                (Yp, var) = model.predict(Xtest)
                model_lock.release()
            except Exception as e:
                rospy.logwarn("Errore while predicting")
		print e
            else:
#		print Yp
                if Yp[0][0] != prev_Yp and Yp > POSITIVE_THRESHOLD:
                    prev_Yp = Yp[0][0]

                    msg = ActionFailure()
                    msg.stamp = rospy.Time.now()
                    msg.cause = "autodetected"
                    signal_pub.publish(msg)
                    trace_pub.publish(current_scan_window_data)
                    print " Sent failure_signal", Yp[0][0]

                    # wait in order to not send too many failure signals
                    time.sleep(2)

        rate.sleep()


    rospy.spin()
