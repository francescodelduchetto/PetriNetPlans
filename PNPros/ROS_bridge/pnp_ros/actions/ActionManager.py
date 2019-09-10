import os
import sys
import rospy
import inspect
import fnmatch
from importlib import import_module
from AbstractAction import AbstractAction
from pnp_msgs.msg import PNPResult, PNPGoal

try:
    sys.path.append(os.environ["PNP_HOME"] + '/scripts')
except:
    print "Please set PNP_HOME environment variable to PetriNetPlans folder."
    sys.exit(1)

import pnp_common
from pnp_common import *

class ActionManager():

    def __init__(self):
        self._action_instances = {}
        self._implemented_actions = []

        actions_found = self._search_actions()

        for action_class in actions_found:
            self._implemented_actions.append(action_class)

        # Start interrupted_goal topic
        self._interrupted_goal_publisher = rospy.Publisher('interrupted_goal', PNPGoal, queue_size=10, latch=True)

    @staticmethod
    def _search_actions():
        actions_found = []
        # Find all the actions implemented
        directory = os.path.dirname(os.path.abspath(__file__))
        for file in [os.path.join(dirpath, f)
                    for dirpath, _, files in os.walk(directory, followlinks=True)
                    for f in fnmatch.filter(files, '*.py')]:
            module_name = os.path.splitext(os.path.basename(file))[0]
            package_name = os.path.dirname(os.path.relpath(file, directory)).replace("/", ".")
            action_class = ActionManager._find_action_implementation(package_name, module_name)
            if action_class:
                actions_found.append(action_class)
                rospy.loginfo("Found implemented action " + module_name)

        return actions_found

    def get_actions(self):
        return [action.__name__ for action in self._implemented_actions]

    def start_action(self, goalhandler):
        goal = goalhandler.get_goal()

        # search for an implementation of the action
        action = None
        for action_impl in self._implemented_actions:
            if action_impl.__name__ == goal.name:
                action = action_impl
                break

        if action is not None:
            # accept the goal
            goalhandler.set_accepted()

            # print "Is goal reached", action.is_goal_reached(goal.params.split("_"))
            params_list = []
            if goal.params != "":
                params_list = goal.params.split("_")

            ## check that the action goal is not already reached
            if not action.is_goal_reached(params_list):
                # Instantiate the action
                action_instance = action(goalhandler, params_list)

                # add action instance to the dict
                self._action_instances.update({
                    goal.id : action_instance
                })

                print "[AM] Starting " + goal.name + " " + goal.params + " " + str(goal.id)

                # start the action
                self._action_instances[goal.id].start_action()
            else:
                print "[AM] Stopping " + goal.name + " " + goal.params + " " + str(goal.id)
                # send the result
                rospy.set_param(get_robot_key(PARAM_PNPACTIONSTATUS) + goalhandler.get_goal().name, ACTION_SUCCESS)

                result = PNPResult()
                result.result = 'OK'
                goalhandler.set_succeeded(result, 'OK')
        else:
            rospy.set_param(get_robot_key(PARAM_PNPACTIONSTATUS) + goalhandler.get_goal().name, ACTION_NOT_IMPLEMENTED)
            rospy.logwarn("action " + goal.name + " not implemented")

    def interrupt_action(self, goalhandler):
        ''' Action interrupted before it finished the execution '''
        goal = goalhandler.get_goal()
        print "[AM] Interrupting " + goal.name + " " + goal.params + " " + str(goal.id)

        # accept the goal
        goalhandler.set_accepted()


        # stop the action
        if goal.id in self._action_instances.keys():
            self._action_instances[goal.id].interrupt_action()

            # publish interrupted goal
            self._interrupted_goal_publisher.publish(goal)

            # remove instance
            del self._action_instances[goal.id]

    def end_action(self, goalhandler):
        ''' Action ended its execution (this is called after the action is already finished, why?)'''
        goal = goalhandler.get_goal()
        print "[AM] Ending " + goal.name + " " + goal.params

        # accept the goal
        goalhandler.set_accepted()

        # end the action
        if goal.id in self._action_instances.keys():

            # remove instance
            del self._action_instances[goal.id]

    @staticmethod
    def is_goal_reached(action_name, parameters):
        # search for an implementation of the action
        action = None
        actions_found = ActionManager._search_actions()
        for action_found in actions_found:
            if action_found.__name__ == action_name:
                action = action_found
                break

        if action is not None:
            # Instantiate the action
            return action.is_goal_reached(parameters)
        else:
            return False

    @staticmethod
    def _find_action_implementation(package_name, module_name):
        # action_class = getattr(import_module(action_name), action_name)
        try:
            if package_name == "":
                full_name = module_name
            else:
                full_name = package_name + "." + module_name
            action_class = getattr(import_module(full_name, package=package_name), module_name)
        except (ImportError, AttributeError) as e:
            rospy.logwarn("action " + module_name + " not implemented")
            print e
            pass
        except Exception as e:
            rospy.logwarn("Exception instantiating action %s: %s" % (module_name, e))
            pass
        else:
            try:
                if issubclass(action_class, AbstractAction) and not inspect.isabstract(action_class):
                    return action_class
                else:
                    rospy.logwarn("class " + action_class.__name__ + " must inherit from AbstractAction")
                    return None
            except TypeError as e:
                rospy.logwarn("class " + action_class.__name__ + " must inherit from AbstractAction")
                pass
