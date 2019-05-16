#!/usr/bin/python
# -*- coding: utf-8 -*-

# ROS action_cmd
import sys
import os
import roslib, rospy
import time
import string
import random
import pnp_msgs.msg, pnp_msgs.srv

import std_msgs.msg


# from PetriNetPlans/pyPNP
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print "Please set PNP_HOME environment variable to PetriNetPlans folder."
    sys.exit(1)

import pnp_cmd_base
from pnp_cmd_base import *

import pnp_common
from pnp_common import *

roslib.load_manifest('pnp_ros')
PKG = 'pnp_ros'
NODE = 'pnp_cmd'

# ROS names (see pnp_ros/include/pnp_ros/names.h)


class PNPCmd(PNPCmd_Base):

    def __init__(self):
        PNPCmd_Base.__init__(self)
        self.pub_actioncmd = None
        self.pub_plantoexec = None
        self._current_action_starttime = None


    def init(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("-a", type=str, default="",
                            help="action name")
        parser.add_argument("-p", type=str, default="",
                            help="params")
        parser.add_argument("-c", type=str, default="",
                            help="command (start, end, interrupt)")
        args = parser.parse_args()
        action = args.a
        params = args.p
        cmd = args.c

        return [action, params, cmd]

    def begin(self, node_name=None):
        if node_name is None:
            node_name = 'plan_' #+ ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(10))
        rospy.init_node(node_name)

        rospy.on_shutdown(self.terminate)

        self._current_action = None

        self.rate = rospy.Rate(2) # hz
        self.rate.sleep()

        key = TOPIC_PNPACTIONCMD #get_robot_key(TOPIC_PNPACTIONCMD)
        self.pub_actioncmd = rospy.Publisher(key, std_msgs.msg.String, queue_size=10)
        print("Publisher %s" %key)
        self.rate.sleep()

        key = TOPIC_PLANTOEXEC #get_robot_key(TOPIC_PLANTOEXEC)
        self.pub_plantoexec = rospy.Publisher(key, std_msgs.msg.String, queue_size=10)
        print("Publisher %s" %key)
        self.rate.sleep()

        key = SRV_PNPCONDITIONEVAL #get_robot_key(SRV_PNPCONDITIONEVAL)
        print("Waiting for service %s ..." %key)
        rospy.wait_for_service(key)
        print("Service %s OK" %key)

        # self.plan_folder = rospy.get_param(PNPPLANFOLDER) #get_robot_key(PNPPLANFOLDER))

        # wait for connections on action_cmd topic
        conn = self.pub_actioncmd.get_num_connections()
        rospy.loginfo('Connections: %d', conn)
        while conn==0:
            # TODO maybe this helps: declare again the publisher
            key = TOPIC_PNPACTIONCMD #get_robot_key(TOPIC_PNPACTIONCMD)
            self.pub_actioncmd = rospy.Publisher(key, std_msgs.msg.String, queue_size=10)
            print("Publisher %s" %key)
            self.rate.sleep()

            conn = self.pub_actioncmd.get_num_connections()
            rospy.loginfo('Connections: %d', conn)

    def end(self):
        rospy.loginfo("Plan is ended")
        self.terminate()

    def terminate(self):
        if self._current_action is not None:
            rospy.logwarn("Terminating action " + str(self._current_action[0]))
            self.action_cmd(self._current_action[0], self._current_action[1], "stop")
            time.sleep(0.5)
        else:
            rospy.logwarn("No action is currently running to be terminated")
        os._exit(os.EX_OK)

    def action_cmd(self,action,params,cmd):
        if (cmd=='stop'):
            cmd = 'interrupt'
            self._current_action_starttime = None
            self._current_action = None
        elif (cmd=='start'):
            self._current_action_starttime = time.time()
            self._current_action = [action, params]
            # remove parameter associated with the action before strting it
            # key = get_robot_key(PARAM_PNPACTIONSTATUS)+action
            # try:
            #     rospy.delete_param(key)
            # except KeyError:
            #     pass
            # the PNP action server will change the status to running after it stated
            # the action. Therefore we wait here for that to happen.
            self.set_action_status(action, ACTION_STARTED)

       # print "ACTIONCMD", action+"_"+params+" "+cmd
        data = action+"_"+params+" "+cmd
        self.pub_actioncmd.publish(data)

    def set_action_status(self, action, status):
        key = get_robot_key(PARAM_PNPACTIONSTATUS)+action
        try:
            r = rospy.set_param(key, status)
            # print "KEY: ", key
            #print('Action %s status %s' %(action,r))
        except Exception as e:
            print "action %s status Exception for parameter: %s" %(action,e)
            r = ''
        return r

    def action_status(self, action):
        key = get_robot_key(PARAM_PNPACTIONSTATUS)+action
        try:
            r = rospy.get_param(key)
            # print "KEY: ", key
            #print('Action %s status %s' %(action,r))
        except KeyError as e:
            print "action %s status KeyError for parameter: %s" %(action,e)
            r = ''
        return r

    def action_starttime(self, action):
        if self._current_action_starttime is None:
            rospy.logwarn("Current action starttime not set.")
        return self._current_action_starttime

    def get_condition(self, cond):
        try:
            service = rospy.ServiceProxy(get_robot_key(SRV_PNPCONDITIONEVAL), pnp_msgs.srv.PNPCondition)
            r = service(cond)
            return r.truth_value!=0
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def plan_cmd(self, planname, cmd): # non-blocking
        if (cmd=='start'):
            self.plan_gen(planname)
            self.pub_plantoexec.publish(planname)
        elif (cmd=='stop'):
            self.pub_plantoexec.publish('stop')
        else:
            print("ERROR: plan cmd %s %s undefined!" %(planname,cmd))
        self.rate.sleep()

def main():
    a = PNPCmd()
    [action, params, cmd] = a.init()
    a.begin()
    a.action_cmd(action, params, cmd)
    a.end()

if __name__ == "__main__":
    main()
