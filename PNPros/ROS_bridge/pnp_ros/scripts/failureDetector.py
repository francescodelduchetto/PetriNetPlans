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

def save_new_trajectory(type="positive", failure_trace=None):
    global current_scan_window_data

    folder = '%s/workspaces/museum_ws/data/detect_trajectories'  % os.path.expanduser("~")
    if type == "negative":
        scan_window = current_scan_window_data
        filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())
    elif type == "falsepositive" and failure_trace != None:
        # take the trace that led to the false positive failure
        scan_window = failure_trace
        filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())
    elif type == "positive":
        scan_window = current_scan_window_data
        filename = '%s/%s.traj' % (folder, rospy.Time.now().to_nsec())

    if scan_window:
        pickle.dump(scan_window.data, open(filename, "wb"))
        rospy.loginfo("Saving failed trajectory in %s" % filename)
    else:
        rospy.logwarn("Scan window not received")

def load_model(_):
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

def ask_failure_confirmation():
    # Ask for confirmation by human
    window = tk.Tk()
    confirmed = None

    def confirm_y():
        confirmed = True
        window.destroy()

    def confirm_n():
        confirmed = False
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

    print "confirmed: ", confirmed
    return confirmed


if __name__ == "__main__":
    #global current_scan_window
    #global model

    load_model("")

    # init node
    rospy.init_node("failure_detector")

    # scan history Subscriber
    rospy.Subscriber("scan_history", Float64MultiArray, receive_scan_history)

    # failure model updated
    #rospy.Subscriber("failure_model_updated", String, load_model)

    # failure Publisher
    signal_pub = rospy.Publisher("failure_signal", ActionFailure, latch=True, queue_size=10)

    ## failure trace Publisher
    #trace_pub = rospy.Publisher("failure_trace", Float64MultiArray, latch=True, queue_size=10)

    # GUI for human signaling
    window = tk.Tk()

    def human_signal_failure():
        # save the failure trajectory
        save_new_trajectory()
        # publish the failure
        msg = ActionFailure()
        msg.stamp = rospy.Time.now()
        msg.cause = "human"
        signal_pub.publish(msg)
        # update the model
        load_model()

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

            try:
                # acquire the lock on the model
                model_lock.acquire()
                (Yp, var) = model.predict(Xtest)
                model_lock.release()
            except:
                rospy.logwarn("Errore while predicting")
            else:
                if Yp[0][0] != prev_Yp and Yp > 0.75:
                    prev_Yp = Yp[0][0]

                    # save current scan window
                    failure_window = current_scan_window

                    # Ask for failure confirmation to humans
                    failure = ask_failure_confirmation()

                    if failure:
                        msg = ActionFailure()
                        msg.stamp = rospy.Time.now()
                        msg.cause = "autodetected"
                        signal_pub.publish(msg)
                        print " Sent failure_signal"
                        #trace_pub.publish(current_scan_window_data)
                    else:
                        #save false positive window
                        save_new_trajectory(type="falsepositive", failure_trace=failure_window)
                        #update the model
                        load_model()

        rate.sleep()


    rospy.spin()
