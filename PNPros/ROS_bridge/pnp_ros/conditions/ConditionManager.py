import os
import glob
import rospy
import rosbag
import inspect

from AbstractCondition import AbstractCondition
from AbstractTopicCondition import AbstractTopicCondition
from importlib import import_module

class ConditionManager():

    def __init__(self):
        self._condition_instances = {}

        # Initialize all the classes in current folder which implement AbstractCondition
        for file in glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "*.py")):
            name = os.path.splitext(os.path.basename(file))[0]
            try:
                condition_class = getattr(import_module(name), name)
            except (ImportError, AttributeError) as e:
		rospy.logwarn("Error while initializing " + name + ": " + str(e))
                continue
            else:
                if issubclass(condition_class, AbstractCondition) and not inspect.isabstract(condition_class):
                    # Instanciate the condition
                    condition_instance = condition_class()

                    self._condition_instances.update({
                        name : condition_instance
                    })

                    rospy.loginfo("Initialized condition " + name)
                else:
                    rospy.logwarn("Class " + name + " does not inherit from AbstractCondition")

    def evaluate(self, condition_name, params):
        try:
            res = self._condition_instances[condition_name].evaluate(params)
            #rospy.loginfo("Evaluating condition " + condition_name + " " + str(params) + ": " + str(res))
            return res
        except KeyError:
            rospy.logwarn("Condition " + condition_name + " not implemented")
            # return true when the condition is not implemented, to avoid loops..
            return True

    def get_value(self, condition_name):
        try:
            res = self._condition_instances[condition_name].get_value()
            #rospy.loginfo("Geting value of condition " + condition_name + ": " + res)
            return res
        except KeyError:
            rospy.logwarn("Condition " + condition_name + " not implemented")
            # return true when the condition is not implemented, to avoid loops..
            return None

    def register_condition_listener(self, listener):
        for (cond_name, cond_instance) in self._condition_instances.items():
            # Register this class as updater listener
            if issubclass(cond_instance.__class__, AbstractTopicCondition):
                cond_instance.register_updates_listener(listener)
                rospy.loginfo(listener.__class__.__name__ + " registered as listener of "\
                            + cond_name)

    # Return a list with the current state of all the conditions
    def get_conditions_dump(self):
        condition_dump = []
        for (cond_name, cond_instance) in self._condition_instances.items():
            condition_dump.append(cond_name + "_" + str(cond_instance.get_value()))

        return condition_dump

    #def get_condition_value(self, cond_name):
    #    cond_value = None
    #    if cond_name in self._condition_instances.keys():
    #        cond_value = str(cond_instance.get_value())

    #    return cond_value
