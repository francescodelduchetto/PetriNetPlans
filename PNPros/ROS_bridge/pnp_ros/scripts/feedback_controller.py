#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from pnp_msgs.msg import ActionFailure


if __name__ == "__main__":
    rospy.init_node("KEYBOAD_FEED_CONTROLLER")

    rate = rospy.Rate(10)

    pub = rospy.Publisher("failure_signal", ActionFailure, queue_size=10, latch=True)



    while not rospy.is_shutdown():
        raw_command = raw_input("raw command: ")
        print str(raw_command)
        if str(raw_command) == "p":
            msg = ActionFailure()
            msg.stamp = rospy.Time.now()
            msg.cause = "human"
        elif str(raw_command) == "n":
            msg = ActionFailure()
            msg.stamp = rospy.Time.now()
            msg.cause = "save"

        pub.publish(msg)

        rate.sleep()




    rospy.spin()
