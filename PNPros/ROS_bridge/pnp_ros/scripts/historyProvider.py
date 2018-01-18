#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Float64MultiArray

QUEUE_SIZE = 20
NUM_LASER_POINTS = 60

scan_queue = [[0.] * NUM_LASER_POINTS] * QUEUE_SIZE

def receive_scan(data):
    scan_queue.pop(0)
    all_scan = list(data.ranges)
    num_ranges = len(data.ranges)
    reduced_scan = all_scan[1:num_ranges:int(np.floor(num_ranges/NUM_LASER_POINTS))]
    scan_queue.append(reduced_scan)

if __name__ == "__main__":
    # init node
    rospy.init_node("history_provider")

    # laserScan listener
    rospy.Subscriber("scan", LaserScan, receive_scan)

    # history Publisher
    pub = rospy.Publisher("scan_history", Float64MultiArray, queue_size=10)

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
        array.data = [point for scan in scan_queue for point in scan]

        pub.publish(array)

        rate.sleep()

    rospy.spin()
