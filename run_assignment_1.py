from typing import List, Tuple, Set, Dict, Optional, cast
from environments.environment_abstract import Environment, State
from environments.farm_grid_world import FarmState
from heapq import heappush, heappop
import time
import pdb
import ctypes
import operator

class Node:
    def __init__(self, state: State, path_cost: float, parent_action: Optional[int], parent, depth):
        self.state: State = state
        self.parent: Optional[Node] = parent
        self.path_cost: float = path_cost
        self.parent_action: Optional[int] = parent_action
        self.depth: int = depth

    def __hash__(self):
        return self.state.__hash__()

    def __gt__(self, other):
        return self.path_cost < other.path_cost

    def __eq__(self, other):
        return self.state == other.state


def get_next_state_and_transition_cost(env: Environment, state: State, action: int) -> Tuple[State, float]:
    rw, states_a, _ = env.state_action_dynamics(state, action)
    state: State = states_a[0]
    transition_cost: float = -rw

    return state, transition_cost


def visualize_bfs(viz, closed_set: Set[State], queue: List[Node], wait: float):
    grid_dim_x, grid_dim_y = viz.env.grid_shape
    for pos_i in range(grid_dim_x):
        for pos_j in range(grid_dim_y):
            viz.board.itemconfigure(viz.grid_squares[pos_i][pos_j], fill="white")

    for state_u in closed_set:
        pos_i_up, pos_j_up = state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="red")

    for node in queue:
        state_u: FarmState = cast(FarmState, node.state)
        pos_i_up, pos_j_up = state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="grey")

    viz.window.update()
    time.sleep(wait)


def visualize_dfs(viz, popped_node: Node, lifo: List[Node]):
    grid_dim_x, grid_dim_y = viz.env.grid_shape
    for pos_i in range(grid_dim_x):
        for pos_j in range(grid_dim_y):
            viz.board.itemconfigure(viz.grid_squares[pos_i][pos_j], fill="white")

    for node in lifo:
        state_u: FarmState = cast(FarmState, node.state)
        pos_i_up, pos_j_up = state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="grey")

    node_parent = popped_node.parent
    while node_parent is not None:
        parent_state_u: FarmState = cast(FarmState, node_parent.state)
        pos_i_up, pos_j_up = parent_state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="red")
        node_parent = node_parent.parent

    viz.window.update()

# Function to produce a list of the solution from the child node to the start state
def get_soln(goal: Node, start_state: State) -> List[int]:
    # Initialize the path list
    path = []
    while goal.state != start_state:
        path.append(goal.parent_action)
        goal = goal.parent


    # print('path ', path)
    return reversed(path)

def breadth_first_search(start_state: State, env: Environment, viz) -> Optional[List[int]]:
# Breadth-first search
# 0: up, 1: down, 2: left, 3: right

    if env.is_terminal(start_state):
        return []
    open = list()
    closed = set()
    path_test= []

    root = Node(start_state, 0, None, None, 0)

    open.append(root)
    closed.add(root.state)
    while open:
        parent = open.pop(0)
        actions = env.get_actions(parent.state)
        for action in actions:
            [child_state, cost] = get_next_state_and_transition_cost(env, parent.state, action)
            #
            child = Node(child_state, parent.path_cost + cost, action, parent, parent.depth +1)

            if env.is_terminal(child.state):
                return get_soln(child, start_state)
            if child.state not in closed:
                closed.add(child.state)
                open.append(child)
                path_test.append(child.parent_action)
            visualize_bfs(viz, closed, open, .01)
        # pdb.set_trace()

    return None



def depth_limited_search(start_state: State, env: Environment, limit: int, viz) -> Optional[List[int]]:
    # Depth-limited search
    if env.is_terminal(start_state):
        return []
    open = list()
    closed = set()
    path_test= []

    root = Node(start_state, 0, None, None, 0)

    open.append(root)
    # closed.add(root.state)
    while open:
        parent = open.pop()
        actions = env.get_actions(parent.state)
        for action in actions:
            [child_state, cost] = get_next_state_and_transition_cost(env, parent.state, action)
            #
            child = Node(child_state, parent.path_cost + cost, action, parent, parent.depth +1)

            if env.is_terminal(child.state):
                return get_soln(child, start_state)
            if child.depth < limit and not is_cycle(child):
                # closed.add(child.state)
                open.append(child)
            visualize_dfs(viz, parent, open)
        # pdb.set_trace()

    return None

def is_cycle(child: Node) -> Optional[List[int]]:
    # parent= None

    parent = child.parent
    if child.state == parent.state:
        return True
    else:
        return False

def iterative_deepening_search(start_state: State, env: Environment, viz) -> List[int]:
    soln = None
    limit = 0
    while not soln:
        soln = depth_limited_search(start_state, env, limit,viz)
        limit += 1

    return soln

def best_first_search(start_state: State, env: Environment, weight_g: float, weight_h: float,
                      viz) -> Optional[List[int]]:



    if env.is_terminal(start_state):
        return []
    open = {}
    closed = {}
    cost_saved_in_closed=0

    target_state_coordinate = start_state.goal_idx
    root = Node(start_state, 0, None, None, 0)

    open[100] = root
    closed[0] = (root.state)
    print('closed', closed)
    while open:
        print('open', open)
        HP = max(open, key=open.get)
        parent = open.pop(HP)
        print('parent_HP',HP)
        actions = env.get_actions(parent.state)
        print('actions',actions)
        for action in actions:
            [child_state, cost] = get_next_state_and_transition_cost(env, parent.state, action)
            child = Node(child_state, parent.path_cost + cost, action, parent, parent.depth +1)
            if env.is_terminal(child.state):
                return get_soln(child, start_state)

            current_state_coordinate = child.state.agent_idx
            h_tuple = tuple(map(operator.sub, current_state_coordinate, target_state_coordinate))
            h = sum([abs(ele) for ele in h_tuple])
            f = weight_g * child.path_cost + weight_h * h

            if child.state not in list(closed.values()): # i did not see the point addidn the argument for the cost: or child.path_cost < [key in key, value closed.ietems() if closed[value] == child.path_cost]:
                closed[child.path_cost] = child.state
                open[f] = child




    return None
