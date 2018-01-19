#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Float64MultiArray

QUEUE_SIZE = 25
GRID_WIDTH = 0
GRID_HEIGHT = 0
NUM_GRID_CELLS = 0

n = 0
time = 0
scan_queue = []

def receive_update(data):
    global n, time, scan_queue
#    if n==0:
#        time = rospy.Time.now().to_sec()
#    elif n==QUEUE_SIZE:
#        now = rospy.Time.now().to_sec()
#        print "Elapsed time ", str(now - time)
#        time = now
#        n =  -1
#    n += 1
    scan_queue.pop(0)
    all_cells = np.array(scan_queue[-1])
    all_cells = np.reshape(all_cells, (GRID_HEIGHT, GRID_WIDTH))
    all_cells[data.y:, data.x:] = np.reshape(data.data, (GRID_HEIGHT - data.y, GRID_WIDTH - data.x))
    scan_queue.append(all_cells.flatten().tolist())

if __name__ == "__main__":
    # init node
    rospy.init_node("history_provider")

    # get the first latch message of the costmap to get the initial map info
    msg = rospy.wait_for_message("/move_base/local_costmap/costmap", OccupancyGrid, timeout=5)
    GRID_WIDTH = msg.info.width
    GRID_HEIGHT = msg.info.height
    NUM_GRID_CELLS = msg.info.width * msg.info.height
    scan_queue = [msg.data] * QUEUE_SIZE

    # get costmap updates with listener
    rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate, receive_update)

    # history Publisher
    pub = rospy.Publisher("scan_history", Float64MultiArray, queue_size=10)

    # specification of array dimentions
    dim1 = MultiArrayDimension()
    dim1.label = "samples"
    dim1.size = QUEUE_SIZE
    dim1.stride = QUEUE_SIZE * NUM_GRID_CELLS
    dim2 = MultiArrayDimension()
    dim2.label = "laser_points"
    dim2.size = NUM_GRID_CELLS
    dim2.stride = NUM_GRID_CELLS

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
