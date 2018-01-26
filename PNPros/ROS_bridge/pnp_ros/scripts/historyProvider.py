#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import threading

from sensor_msgs.msg import LaserScan
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Float64MultiArray

QUEUE_SIZE = 10
NUM_LASER_POINTS = 30

n = 0
time = 0
scan_queue = [[0.] * NUM_LASER_POINTS] * QUEUE_SIZE
queue_lock = None

def receive_scan(data):
#    global n, time
#    if n==0:
#        time = rospy.Time.now().to_sec()
#    elif n==QUEUE_SIZE:
#        now = rospy.Time.now().to_sec()
#        print "Elapsed time ", str(now - time)
#        time = now
#        n =  -1
#    n += 1
    global queue_lock, scan_queue
    all_scan = list(data.ranges)
    num_ranges = len(data.ranges)
    step = int(np.floor(float(num_ranges)/NUM_LASER_POINTS))
    reduced_scan = []
    for i in range(0, num_ranges, step):
        reduced_scan.append(min(all_scan[i:i+step]))
    reduced_scan = reduced_scan[:NUM_LASER_POINTS]
    #reduced_scan = all_scan[1:num_ranges:int(np.floor(num_ranges/NUM_LASER_POINTS))]
    queue_lock.acquire()
    scan_queue.pop(0)
    scan_queue.append(reduced_scan)
    queue_lock.release()

if __name__ == "__main__":
    # init node
    rospy.init_node("history_provider")

    # laserScan listener
    rospy.Subscriber("scan360", LaserScan, receive_scan)

    # history Publisher
    pub = rospy.Publisher("scan_history", Float64MultiArray, queue_size=10)

    queue_lock = threading.Lock()

    # specification of array dimentions
    dim1 = MultiArrayDimension()
    dim1.label = "samples"
    dim1.size = QUEUE_SIZE
    dim1.stride = QUEUE_SIZE * NUM_LASER_POINTS
    dim2 = MultiArrayDimension()
    dim2.label = "laser_points"
    dim2.size = NUM_LASER_POINTS
    dim2.stride = NUM_LASER_POINTS

    dimentions = [dim1, dim2]

    # specification of array layout
    layout = MultiArrayLayout()
    layout.dim = dimentions
    layout.data_offset = 0

    rate = rospy.Rate(10) #hz
    while not rospy.is_shutdown():
        # create new array
        array = Float64MultiArray()
        array.layout = layout
	queue_lock.acquire()
        array.data = [point for scan in scan_queue for point in scan]
	queue_lock.release()
        pub.publish(array)

        rate.sleep()

    rospy.spin()
