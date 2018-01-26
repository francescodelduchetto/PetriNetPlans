import os
import rospy, rosbag
import multiprocessing
from AbstractTopicCondition import ConditionListener
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStartStateActionSaverResponse,\
                         PNPStopStateActionSaver, PNPStopStateActionSaverResponse

class StateActionPairGenerator(ConditionListener):

    def __init__(self, action_manager, condition_manager):
        self._action_manager = action_manager
        self._condition_manager = condition_manager

        self._actions_history = {}
        self._states_history = {}
        self._saving_files = {}
        self._saving_bags = {}
        self._request_info = {}
        self._files_lock = multiprocessing.Lock()
        self._bags_lock = multiprocessing.Lock()

        # Initialize the state action saver services
        self._start_sa_service_provider = rospy.Service("start_state_action_saver",
            PNPStartStateActionSaver, self._start_state_action_saver_cb)
        self._stop_sa_service_provider = rospy.Service("stop_state_action_saver",
            PNPStopStateActionSaver, self._stop_state_action_saver_cb)
        self._check_running_service_provider = rospy.Service("running_state_action_saver",
            PNPStopStateActionSaver, self._check_running_cb)

        self._condition_manager.register_condition_listener(self)

    def _check_running_cb(self, req):
        goal_goal = req.goal

        if goal_goal in self._saving_files.keys():
            return PNPStopStateActionSaverResponse(1)
        else:
            return PNPStopStateActionSaverResponse(0)

    def _start_state_action_saver_cb(self, req):
        goal = req.goal
        filepath = req.filepath
        save_bag = req.save_bag
        state_conditions = req.state_conditions
        action_conditions = req.action_conditions
        print len(state_conditions), action_conditions

        # Start saving the action-state pairs
        if not goal in self._saving_files.keys():
            if filepath == "":
                # create new file
                file_name = goal + "_" + str(rospy.Time.now().to_nsec()) + ".txt"
                filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)),\
                        "../demonstrations", file_name)
            file = open(filepath, "w")

            # insert file in dict
            self._saving_files.update({
                goal : file
            })
            self._actions_history.update({
                goal : []
            })
            self._states_history.update({
                goal : []
            })
            self._request_info.update({
                goal : {
                    "state_conditions": state_conditions,
                    "action_conditions": action_conditions
                }
            })

            rospy.loginfo("Started recording demonstration in " + filepath)

        else:
            rospy.logwarn("Saver for " + goal + " already running")
            return PNPStartStateActionSaverResponse(0)

        # Start saving in the bag
        if save_bag:
            if not goal in self._saving_bags.keys():
                # create new bag
                file_name = goal + "_" + str(rospy.Time.now().to_nsec()) + ".bag"
                folder = '%s/workspaces/museum_ws/data/bagfiles'  % os.path.expanduser("~")
                filepath = os.path.join(folder, file_name)
                bag = rosbag.Bag(filepath, "w")

                # insert file in dict
                with self._bags_lock:
                    self._saving_bags.update({
                        goal : bag
                    })

                rospy.loginfo("Started dumping demonstration in bag " + filepath)
            else:
                rospy.logwarn("Bag saver for " + goal + " already running")
                return PNPStartStateActionSaverResponse(0)

        return PNPStartStateActionSaverResponse(1)

    def _stop_state_action_saver_cb(self, req):
        goal = req.goal

        # Stop the state-action pair saver
        if goal in self._saving_files.keys():
            # Save state actions in all open files
            file = self._saving_files[goal]
            for timestep in range(len(self._states_history[goal])):
                if len(self._states_history[goal][timestep]) < len(self._request_info[goal]["state_conditions"]) or \
			len(self._actions_history[goal][timestep]) < len(self._request_info[goal]["action_conditions"]):
                    continue
                for i, state_el in enumerate(self._states_history[goal][timestep]):
                    if i>0:
                        file.write("  ")
                    file.write(state_el)
                file.write("\t")
                for i, ex_action in enumerate(self._actions_history[goal][timestep]):
                    if i>0:
                        file.write("  ")
                    file.write(ex_action)
                file.write("\n")

            # close the file
            with self._files_lock:
                self._saving_files[goal].close()

                # remove the file from the dict
                del self._saving_files[goal]
                del self._states_history[goal]
                del self._actions_history[goal]
                del self._request_info[goal]

                rospy.loginfo("Closed file " + goal)
        else:
            rospy.logwarn("No running saver in file " + goal + " found")
            return PNPStopStateActionSaverResponse(0)

        # Stop the bag saving
        if goal in self._saving_bags.keys():
            with self._bags_lock:
                self._saving_bags[goal].close()
                del self._saving_bags[goal]
                rospy.loginfo("Closed bag for " + goal)
        else:
            rospy.logwarn("No running bag saver for " + goal + " found")

        return PNPStopStateActionSaverResponse(1)

    def receive_update(self, condition_instance):
        condition_name = condition_instance.get_name()
        condition_value = condition_instance.get_value()

        # Save in state-action pairs file
        with self._files_lock:
            for goal_id in self._saving_files.keys():
                # Get the current condition state
                #current_state = self._condition_manager.get_conditions_dump()

                # take the requested state conditions
                state_conds = self._request_info[goal_id]["state_conditions"]
                # take the requested action conditions
                action_conds = self._request_info[goal_id]["action_conditions"]

                action_candidates = []
                # consider this condition if is in the list or if the list is empty (all the conditions)
                if condition_name in state_conds or len(state_conds) == 0:
                    # if the condition was updated in the last timestep, here we create a new timestep
                    if len(self._states_history[goal_id]) == 0 or condition_name in [cond.split("_")[0] for cond in self._states_history[goal_id][-1]]:
                        # Save the condition
                        self._states_history[goal_id].append([condition_name + "_".join(["", condition_value])])

                        # Add empty container of actions for the current step
                        self._actions_history[goal_id].append([])
                    else:
                        # Save the condition
                        self._states_history[goal_id][-1].append(condition_name + "_".join(["", condition_value]))
                        # ...and propagate in the previous states if not present
                        for timestep in range(len(self._states_history[goal_id])-2, -1, -1):
                            if not condition_name in [cond.split("_")[0] for cond in self._states_history[goal_id][timestep]]:
                                self._states_history[goal_id][timestep].append(condition_name + "_".join(["", condition_value]))
                            else:
                                break

                    # if we don't specify which conditions represents our action we try to infer them when the state changes
                    if len(action_conds) == 0:
                        # Get executable actions
                        action_list = self._action_manager.get_actions()

                        # Check the actions that reached the goal after this change
                        if condition_value.lower() != "none":
                            for action in action_list:
                                #print action, "goal reached?"
                                if self._action_manager.is_goal_reached(action, condition_value.split("_")):
                                    action_candidates.append(action + "_".join(["", condition_value]))

                # TODO make it working with multiple action conditions
                # save the condition in the action list
                if condition_name in action_conds:
                    action_candidates.append(condition_name + "_".join(["", condition_value]))

                # insert the action at the last state received, remove the previous timesteps without actions
                executed_actions = action_candidates[:]
		if len(self._actions_history[goal_id]) > 0 and len(self._actions_history[goal_id][-1]) == 0:
		    self._actions_history[goal_id][-1] = executed_actions
                for timestep in range(len(self._actions_history[goal_id])-2, -1, -1):
                    if len(self._actions_history[goal_id][timestep]) == 0:
                        del self._actions_history[goal_id][timestep]
			del self._states_history[goal_id][timestep]
                    else:
                        break



        # Save in the bag
        topic_message = condition_instance.get_data() # the actual topic message
        topic_name = condition_instance._topic_name # the type of topic message
        with self._bags_lock:
            for bag in self._saving_bags.values():
                #print topic_name, topic_message
                bag.write(topic_name, topic_message)
