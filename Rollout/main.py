import copy
import gym
import gym_sokoban
import numpy as np

# Credit for this: Nicholas Swift
# as found at https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
from warnings import warn
import heapq


class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        return self.f < other.f

    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        return self.f > other.f


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement=False):
    """
    Returns a list of tuples as a path from the given start to the given end in the given maze
    :param maze:
    :param start:
    :param end:
    :return:
    """

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (10*len(maze[0]) * len(maze) // 2)

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),
                            (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1

        if outer_iterations > max_iterations:
            # if we hit this point return the path such as it is
            # it will not contain the destination
            warn("giving up on pathfinding too many iterations")
            return return_path(current_node)

        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []

        for new_position in adjacent_squares:  # Adjacent squares

            # Get node position
            node_position = (
                current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = abs(child.position[0] - end_node.position[0]) + \
                abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

    warn("Couldn't get a path to destination")
    return None


def pre_processing(state_mat, pos, target):
    # Represent all obstacles with 1, free space with 0
    for i in range(state_mat.shape[0]):
        for j in range(state_mat.shape[1]):
            # If free space or agent
            if state_mat[i][j] == 1 or state_mat[i][j] == 2 or state_mat[i][j] >= 5:
                state_mat[i][j] = 0
            elif state_mat[i][j] == 0 or state_mat[i][j] == 4:  # wall or box
                state_mat[i][j] = 1
    state_mat[target] = 0
    return state_mat


def base_policy(state, return_list, agent_number):
    state_mat = state[0]
    state_targets = state[1]

    agent_id = 5+agent_number

    # Linear search
    for i in range(state_mat.shape[0]):
        for j in range(state_mat.shape[1]):
            if (state_mat[i][j] == agent_id) or (state_mat[i][j] == agent_id+1):
                pos = (i, j)

    processed_state_mat = pre_processing(
        state_mat, pos, state_targets[agent_number])
    path = astar(processed_state_mat, pos, state_targets[agent_number])
    delta_to_action = {
        (-1, 0): 1,
        (1, 0): 2,
        (0, -1): 3,
        (0, 1): 4}

    actions = []
    for i in range(1, len(path)):
        actions.append(
            delta_to_action[(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])])
    return actions if return_list else actions[0]

def rollout(state, agents, tot_reward):
    #for i in range()
    pass

num_of_agents = 2
env = gym.make("Sokoban-v0")
state = env.reset(render_mode="raw", num_of_agents=num_of_agents)
env.render()

reward_tot = 0
done = False
while not done:
    '''
    ac = input()
    ac = ac.split()
    input_list = [int(i) for i in ac]
    target_dict = {(2, 6): "Box 1", (6, 7): "Box 2", (6, 8)
                    : "Box 3", (3, 8): "Drop off point"}
    '''

    ac0 = base_policy(copy.deepcopy(state), False, 0)
    # in i step behöver actions innehålla vad båda ska göra
    ac1 = base_policy(copy.deepcopy(state), False, 1)
    print(ac0, ac1)
    state, reward, done, info = env.step([ac0, ac1], "raw") 
    target_list = state[1]
    
    for i, target in enumerate(target_list):
        print("Agent ", i+1, "target: ", target)
    
    # test
    print("Reward: ", reward)
    print(info)
    input()
    reward_tot += reward
    env.render()
