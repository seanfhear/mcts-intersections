from copy import deepcopy
from passing_order import PassingOrder
import time
import math
import random
import yaml

config_file = open("./config.yaml")
cfg = yaml.safe_load(config_file)


class IntersectionState:
    def __init__(self, veh_data):
        self.current_order = []
        self.veh_data = veh_data
        self.veh_ids = [veh_id for veh_id in veh_data.keys()]

    def get_possible_actions(self):
        """
        Next vehicle that can be selected is the leading vehicle from any inbound edge
        i.e. don't select a vehicle that would have to overtake a vehicle ahead of it on the same edge
        """
        closest_vehs = {}
        for veh in self.veh_ids:
            edge = self.veh_data[veh]["edge"]
            distance = self.veh_data[veh]["distance"]
            if edge in closest_vehs:
                if distance < closest_vehs[edge]["distance"]:
                    closest_vehs[edge] = {
                        "veh": veh,
                        "distance": distance
                    }
            else:
                closest_vehs[edge] = {
                    "veh": veh,
                    "distance": distance
                }

        return [item[1]["veh"] for item in closest_vehs.items()]

    def take_action(self, veh_id):
        new_state = deepcopy(self)

        new_state.current_order.append((
            veh_id, {
                "route": self.veh_data[veh_id]["route"],
                "distance": self.veh_data[veh_id]["distance"],
                "adjustment": self.veh_data[veh_id]["adjustment"]
            }
        ))
        new_state.veh_ids.remove(veh_id)

        return new_state

    def is_terminal(self):
        return len(self.veh_ids) == 0

    def get_reward(self):
        passing_order = PassingOrder(self.current_order)
        passing_order.calculate_adjustments()

        return passing_order.total_adjustment * -1


def random_policy(state):
    while not state.is_terminal():
        try:
            action = random.choice(state.get_possible_actions())
        except IndexError:
            raise Exception("Non-terminal state has no possible actions: " + str(state))
        state = state.take_action(action)

    return state.get_reward(), state.current_order


# https://pypi.org/project/mcts/
# Source code from this python library extracted so it can be modified slightly
class TreeNode:
    def __init__(self, state, parent):
        self.state = state
        self.is_terminal = state.is_terminal()
        self.is_fully_expanded = self.is_terminal
        self.parent = parent
        self.num_visits = 0
        self.total_reward = 0
        self.children = {}


class MCTS:
    """
    This class is taken from the 'mcts' python library and modified to return best result from all simulations
    instead of returning the best next move
    """
    def __init__(self, time_limit=None, iteration_limit=None, exploration_constant=1 / math.sqrt(2),
                 rollout_policy=random_policy):
        if time_limit is not None:
            if iteration_limit is not None:
                raise ValueError("Cannot have both a time limit and an iteration limit")
            # time taken for each MCTS search in milliseconds
            self.time_limit = time_limit
            self.limit_type = 'time'
        else:
            if iteration_limit is None:
                raise ValueError("Must have either a time limit or an iteration limit")
            # number of iterations of the search
            if iteration_limit < 1:
                raise ValueError("Iteration limit must be greater than one")
            self.search_limit = iteration_limit
            self.limit_type = 'iterations'
        self.exploration_constant = exploration_constant
        self.rollout = rollout_policy
        self.root = None
        self.best_order = []
        self.best_reward = float("-inf")

    def search(self, initial_state):
        self.root = TreeNode(initial_state, None)

        if self.limit_type == 'time':
            time_limit = time.time() + self.time_limit / 1000
            while time.time() < time_limit:
                self.execute_round()
        else:
            for i in range(self.search_limit):
                self.execute_round()

        return self.best_order

    def execute_round(self):
        node = self.select_node(self.root)
        reward, order = self.rollout(node.state)

        if reward > self.best_reward:
            self.best_order = order

        self.backpropagate(node, reward)

    def select_node(self, node):
        while not node.is_terminal:
            if node.is_fully_expanded:
                node = self.get_best_child(node, self.exploration_constant)
            else:
                return self.expand(node)
        return node

    def expand(self, node):
        actions = node.state.get_possible_actions()
        for action in actions:
            if action not in node.children.keys():
                new_node = TreeNode(node.state.take_action(action), node)
                node.children[action] = new_node
                if len(actions) == len(node.children):
                    node.is_fully_expanded = True
                return new_node

        raise Exception("Should never reach here")

    def backpropagate(self, node, reward):
        while node is not None:
            node.num_visits += 1
            node.total_reward += reward
            node = node.parent

    def get_best_child(self, node, exploration_value):
        best_value = float("-inf")
        best_nodes = []
        for child in node.children.values():
            node_value = child.total_reward / child.num_visits + exploration_value * math.sqrt(
                2 * math.log(node.num_visits) / child.num_visits)
            if node_value > best_value:
                best_value = node_value
                best_nodes = [child]
            elif node_value == best_value:
                best_nodes.append(child)
        return random.choice(best_nodes)

    def get_action(self, root, best_child):
        for action, node in root.children.items():
            if node is best_child:
                return action


def get_passing_order(veh_ids):
    initial_state = IntersectionState(veh_ids)
    searcher = MCTS(time_limit=cfg["mcts_time"])

    return searcher.search(initial_state=initial_state)
