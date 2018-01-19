#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import roslib
import rospy
import actionlib
sys.path.append(os.path.join(os.path.dirname(__file__), '../actions'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../conditions'))

from ActionManager import ActionManager
from ConditionManager import ConditionManager
from RecoveryActionServer import RecoveryActionServer
from StateActionPairGenerator import StateActionPairGenerator
from pnp_msgs.msg import PNPActionFeedback, PNPResult, PNPAction
from pnp_msgs.srv import PNPCondition, PNPConditionResponse, PNPConditionValue, PNPConditionValueResponse

roslib.load_manifest('pnp_ros')
PKG = 'pnp_ros'
NODE = 'pnpactionserver'
conditionManager = None
actionManager = None

class PNPActionServer(object):
    #  create messages that are used to publish feedback/result
    _feedback = PNPActionFeedback()
    _result = PNPResult()

    def __init__(self, name):
        self._action_server_name = name
        self._as = actionlib.ActionServer(self._action_server_name,
                                          PNPAction,
                                          self.execute_cb,
                                          auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action Server started' % self._action_server_name)

    def execute_cb(self, goalhandler):
        r = rospy.Rate(4)
        # init running
        self._feedback.feedback = 'running...'
        goalhandler.publish_feedback(self._feedback)
        goal = goalhandler.get_goal()

        # publish info to the console for the user
        rospy.loginfo('%s: Action %s %s' %
                      (self._action_server_name, goal.name, goal.params))
        if goal.function == 'start':
            # start executing the action
            actionManager.start_action(goalhandler)
        elif goal.function == 'interrupt':
            # print '### Interrupt ',goal.name
            actionManager.interrupt_action(goalhandler)
        elif goal.function == 'end':
            # print '### End ',goal.name
            actionManager.end_action(goalhandler)

def handle_PNPConditionEval(req):
    cond_elems = req.cond.split("_")
    cond = cond_elems[0]
    params = cond_elems[1:]

    # evaluate through the condition manager
    cond_truth_value = conditionManager.evaluate(cond, params)

    if cond_truth_value:
        rospy.loginfo('Eval condition: ' + cond + ' ' + ' '.join(params) + ' value: ' + str(cond_truth_value))

    return PNPConditionResponse(cond_truth_value)

def handle_PNPConditionValue(req):
    cond = req.cond

    cond_value = str(conditionManager.get_value(cond))

    if cond_value:
        #rospy.loginfo('Condition: ' + cond + ' value: ' + cond_value)
        return PNPConditionValueResponse(cond_value)
    else:
        return PNPConditionValueResponse("None")

if __name__ == '__main__':
    rospy.init_node(NODE)
    rospy.set_param('robot_name', 'dummy')

    conditionManager = ConditionManager()
    actionManager = ActionManager()

    # Service which returns truth value of condition
    rospy.Service('PNPConditionEval',
                  PNPCondition,
                  handle_PNPConditionEval)

    # Service which returns value of condition
    rospy.Service('PNPConditionValue',
                  PNPConditionValue,
                  handle_PNPConditionValue)

    StateActionPairGenerator(actionManager, conditionManager)

    RecoveryActionServer()

    PNPActionServer("PNP")

    rospy.spin()
