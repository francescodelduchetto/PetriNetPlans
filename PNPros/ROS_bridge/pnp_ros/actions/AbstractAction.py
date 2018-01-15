import rospy
import threading

from abc import ABCMeta, abstractmethod
from pnp_msgs.msg import PNPActionFeedback, PNPResult

class AbstractAction():
    __metaclass__ = ABCMeta

    def __init__(self, goalhandler, params):
        self.goalhandler = goalhandler
        self.params = params

        # create event for stopping the action
        self.cancel_event = threading.Event()

    @abstractmethod
    def _start_action(self):
        raise NotImplementedError()

    @abstractmethod
    def _stop_action(self):
        raise NotImplementedError()

    def _is_action_done(self):
        return self.is_goal_reached(self.params)

    @classmethod
    def is_goal_reached(cls, params):
        ''' Static definition of goal reached for the action '''
        raise NotImplementedError()

    ## main execution thread
    def _actionThread_exec(self):
        result = PNPResult()
        feedback = PNPActionFeedback()

        self._start_action()

        # rate 0.5hz
        r = rospy.Rate(2)

        # wait until the action is done
        while not self._is_action_done():
            # request to cancel action
            if self.cancel_event.isSet():
                break

            # send feedback
            feedback.feedback = 'running ...'
            self.goalhandler.publish_feedback(feedback)

            r.sleep()

        self._stop_action()

        # send the result
        result.result = 'OK'
        self.goalhandler.set_succeeded(result, 'OK')


    def start_action(self):
        th = threading.Thread(None, self._actionThread_exec, args=())
        th.start()

    def interrupt_action(self):
        self.cancel_event.set()
