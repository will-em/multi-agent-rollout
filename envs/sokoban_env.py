import gym
from gym.utils import seeding
from gym.spaces.discrete import Discrete
from gym.spaces import Box
from .room_utils import generate_room, generate_static_room
from .render_utils import room_to_rgb, room_to_tiny_world_rgb
import numpy as np
import random
from random import randint


class Agent:
    def __init__(self, agent_id, init_pos, target=None, has_box=False):
        self.id = agent_id
        self.pos = init_pos
        self.target = target
        self.has_box = has_box


class SokobanEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array', 'tiny_human', 'tiny_rgb_array', 'raw']
    }

    def __init__(self,
                 dim_room=(15, 15),
                 max_steps=120,
                 num_boxes=4,
                 num_gen_steps=None,
                 reset=True):

        # General Configuration
        self.dim_room = dim_room
        if num_gen_steps == None:
            self.num_gen_steps = int(1.7 * (dim_room[0] + dim_room[1]))
        else:
            self.num_gen_steps = num_gen_steps

        self.num_boxes = num_boxes
        self.boxes_on_target = 0

        # Penalties and Rewards
        self.penalty_for_step = -1
        self.reward_box_on_target = 1000
        self.reward_finished = 0
        self.reward_last = 0
        self.reward_collision = -10000000

        # Other Settings
        self.viewer = None
        self.max_steps = max_steps
        self.action_space = Discrete(len(ACTION_LOOKUP))
        screen_height, screen_width = (dim_room[0] * 16, dim_room[1] * 16)
        self.observation_space = Box(low=0, high=255, shape=(
            screen_height, screen_width, 3), dtype=np.uint8)

        discount_factor = 0.9

        self.discount_vec = [discount_factor**i for i in range(201)]
        self.boxes = []

    def targetPicker(self, agent, rand=False):
        if len(self.boxes_to_be_picked) > 0:
            if rand:
                target_pos = self.boxes_to_be_picked.pop(
                    random.randrange(len(self.boxes_to_be_picked)))
            else:
                distances = []
                for box in self.boxes_to_be_picked:
                    distances.append(
                        abs(agent.pos[0]-box[0])+abs(agent.pos[1]-box[1]))
                target_pos = self.boxes_to_be_picked.pop(
                    distances.index(min(distances)))

        else:
            if agent.pos == self.drop_off:
                target_pos = ((agent.id+1)//2-1, 1) #End points if upper dropoff
            else:
                target_pos = ((agent.id+1)//2-1, 12) #End points if lower dropoff
        agent.target = target_pos

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, actions, observation_mode='rgb_array'):
        assert observation_mode in ['rgb_array', 'tiny_rgb_array', 'raw']

        self.num_env_steps += 1

        self.new_box_position = None
        self.old_box_position = None

        self.reward_last = 0

        moved_box = []
        self.moved_player = []

        self.old_pos = []
        self.new_pos = []

        positions = dict()
        self.collision = 0

        for action, agent in zip(actions, self.agents):
            assert action in ACTION_LOOKUP

            if action == 0:
                self.old_pos.append(agent.pos)
                self.new_pos.append(agent.pos)
                self.moved_player.append(False)
                moved_box.append(False)
            else:
                p, b = self._move(action, agent)
                moved_box.append(b)
                self.moved_player.append(p)

            old_position = self.old_pos[-1]
            new_position = self.new_pos[-1]
            new_vec = list(positions.values())
            # print("new_vec ", new_vec, new_position)
            if new_position in new_vec:
                self.collision += 1

            if new_position in positions and positions[new_position] == old_position:
                self.collision += 1

            positions[old_position] = new_position

        for pos in self.old_pos:
            if not pos in self.new_pos:
                self.room_state[pos] = self.room_fixed[pos]

        self._calc_reward()

        done = self._check_if_done()

        # Convert the observation to RGB frame
        observation = self.render(mode=observation_mode)

        info = {
            "action.name": actions,
            "action.moved_player": self.moved_player,
            "action.moved_box": moved_box,
        }
        target_list = []
        for agent in self.agents:
            target_list.append(agent.target)
        if done:
            info["maxsteps_used"] = self._check_if_maxsteps()
            info["all_boxes_on_target"] = self._check_if_goal_is_met()

        return [observation, target_list], self.reward_last, done, info

    def _move(self, action, agent):
        """
        Moves the player to the next field, if it is not occupied.
        :param action:
        :return: Boolean, indicating a change of the room's state
        """

        change = CHANGE_COORDINATES[(action - 1) % 4]
        new_position = (agent.pos[0] + change[0], agent.pos[1] + change[1])
        current_position = agent.pos

        # Move player if the field in the moving direction is either
        # an empty field or an empty box target.
        # print(new_position)
        # print(self.room_state[new_position[0], new_position[1]])
        # print(self.room_state[new_position[0], new_position[1]] == 4)

        agent_ids = [5+i for i in range(2*len(self.agents))]

        if self.room_state[new_position[0], new_position[1]] in [1, 2]+agent_ids or new_position == agent.target:

            moved_box = agent.has_box
            agent.pos = new_position
            # print(agent.id, agent.pos)
            if self.room_state[new_position[0], new_position[1]] == 4 or agent.has_box:
                self.room_state[new_position[0], new_position[1]] = agent.id+1
                if not agent.has_box:
                    agent.has_box = True
                    if new_position[0] in [4,5]: #if rows 4 or 5
                        agent.target = self.drop_off
                    elif new_position[0] in [8,9]: #or if rows 8 or 9
                        agent.target = self.drop_off2
                    self.reward_last += 100
            else:
                self.room_state[new_position[0], new_position[1]] = agent.id
            self.old_pos.append(current_position)
            self.new_pos.append(new_position)
            # self.room_state[current_position[0], current_position[1]] = \
            #    self.room_fixed[current_position[0], current_position[1]]
            return True, moved_box

        self.old_pos.append(current_position)
        self.new_pos.append(current_position)

        return False, False

    def _calc_reward(self):
        """
        Calculate Reward Based on
        :return:
        """
        # Every step a small penalty is given, This ensures
        # that short solutions have a higher reward.

        for i, agent in enumerate(self.agents):
            if self.moved_player[i]:
                self.reward_last += self.penalty_for_step
            else:
                self.reward_last += self.penalty_for_step
            if agent.has_box and (agent.pos == self.drop_off or agent.pos == self.drop_off2):
                self.reward_last += self.reward_box_on_target  # * \
                # self.discount_vec[self.num_env_steps]
                self.room_state[agent.pos] = agent.id
                agent.has_box = False
                self.targetPicker(agent)  # Switch target to rest pos

        self.reward_last += self.reward_collision * \
            self.collision*self.discount_vec[self.num_env_steps]

        game_won = self._check_if_goal_is_met()
        if game_won:
            self.reward_last += self.reward_finished  # * \
            # self.discount_vec[self.num_env_steps]

    def _check_if_done(self):
        # Check if the game is over either through reaching the maximum number
        # of available steps or by pushing all boxes on the targets.
        return self._check_if_goal_is_met() or self._check_if_maxsteps() or self.collision > 0

    def _check_if_goal_is_met(self):
        box_check = True
        # Check if there are no boxes left on the field
        for X, Y in self.boxes:
            if(self.room_state[X][Y] == 4):
                box_check = False

        no_box_on_agents = True
        for agent in self.agents:  # Check if there are no boxes on the agents
            if agent.has_box:
                no_box_on_agents = False
        return box_check and no_box_on_agents

    def _check_if_maxsteps(self):
        return (self.max_steps == self.num_env_steps)

    def generate_box_pos(self, rows, num_boxes):
        tuples = []
        while len(tuples) < num_boxes:
            rand = (randint(rows[0], rows[-1]), randint(4, self.dim_room[1]-5)) #------------------------------------------------------------------
            if rand not in tuples and rand[0] in rows:
                tuples.append(rand)

        return tuples

    def reset(self, second_player=False, render_mode='rgb_array', num_of_agents=1, cached_state=None):
        self.num_of_agents = num_of_agents
        self.agents = []
        self.boxes_to_be_picked = []

        self.reward_last = 0

        if cached_state:  # Recreate old cached state
            state_mat = cached_state[0]
            self.room_state = state_mat

            agent_indicies1 = [5+2*i for i in range(self.num_of_agents)]
            agent_indicies2 = [6+2*i for i in range(self.num_of_agents)]
            agent_indicies = agent_indicies1 + agent_indicies2

            for i in range(self.room_state.shape[0]):
                for j in range(self.room_state.shape[1]):
                    if self.room_state[i][j] in agent_indicies:
                        agent_id = self.room_state[i][j] - 1\
                            if self.room_state[i][j] % 2 == 0 else self.room_state[i][j]
                        agent_pos = (i, j)
                        agent_has_box = True if self.room_state[i][j] % 2 == 0 else False
                        agent = Agent(agent_id, agent_pos,
                                      has_box=agent_has_box)
                        self.agents.append(agent)

            self.agents.sort(key=lambda x: x.id)

            state_targets = cached_state[1]
            for i in range(len(self.agents)):
                self.agents[i].target = state_targets[i]

            # Fill boxes to be picked
            for i in range(self.room_state.shape[0]):
                for j in range(self.room_state.shape[1]):
                    if self.room_state[i][j] == 4 and not (i, j) in state_targets:
                        self.boxes_to_be_picked.append((i, j))

            self.num_env_steps = cached_state[2]

        else:
            self.room_fixed = np.ones(
                (self.dim_room[0]-2, self.dim_room[1]-2), dtype=int)
            self.room_fixed = np.pad(self.room_fixed, pad_width=1, mode='constant',
                                     constant_values=0)

            # Extraction points
            self.drop_off = (2, 2)
            self.drop_off2 = (11, self.dim_room[1]-3)
            self.room_fixed[self.drop_off] = 2
            self.room_fixed[self.drop_off2] = 2
            # Shelves
            rows = [4, 5, 8, 9]

            self.boxes = self.generate_box_pos(rows, 10)
            self.boxes_to_be_picked = self.boxes.copy()

            for i in rows:
                for j in range(4, self.dim_room[1]-4):
                    self.room_fixed[i, j] = 0
            # Boxes room_state
            for i in range(len(self.boxes)):
                self.room_fixed[self.boxes[i][0]][self.boxes[i][1]] = 1

            self.room_state = np.copy(self.room_fixed)

            for i in range(len(self.boxes)):
                self.room_state[self.boxes[i][0]][self.boxes[i][1]] = 4

            # Player
            for i in range(self.num_of_agents):
                init_pos = (2, 4+i)
                self.agents.append(Agent(5+2*i, init_pos))
                self.room_state[self.agents[i].pos] = self.agents[i].id
                self.targetPicker(self.agents[i])

            self.box_mapping = {}

            self.num_env_steps = 0

            self.box_on_agent = False

        self.collision = 0
        starting_observation = self.render(render_mode)

        target_list = []
        for agent in self.agents:
            target_list.append(agent.target)
        return starting_observation, target_list

    def render(self, mode='human', close=None, scale=1):
        assert mode in RENDERING_MODES

        img = self.get_image(mode, scale)

        if 'rgb_array' in mode:
            return img

        elif 'human' in mode:
            from gym.envs.classic_control import rendering
            if self.viewer is None:
                self.viewer = rendering.SimpleImageViewer()
            self.viewer.imshow(img)
            return self.viewer.isopen

        elif 'raw' in mode:
            return self.room_state

        else:
            super(SokobanEnv, self).render(
                mode=mode)  # just raise an exception

    def get_image(self, mode, scale=1):

        if mode.startswith('tiny_'):
            img = room_to_tiny_world_rgb(
                self.room_state, self.room_fixed, scale=scale)
        else:
            img = room_to_rgb(
                self.room_state, self.agents, self.room_fixed)

        return img

    def close(self):
        if self.viewer is not None:
            self.viewer.close()

    def set_maxsteps(self, num_steps):
        self.max_steps = num_steps

    def get_action_lookup(self):
        return ACTION_LOOKUP

    def get_action_meanings(self):
        return ACTION_LOOKUP


ACTION_LOOKUP = {
    0: 'no operation',
    1: 'move up',
    2: 'move down',
    3: 'move left',
    4: 'move right',
}

# Moves are mapped to coordinate changes as follows
# 0: Move up
# 1: Move down
# 2: Move left
# 3: Move right
CHANGE_COORDINATES = {
    0: (-1, 0),
    1: (1, 0),
    2: (0, -1),
    3: (0, 1)
}

RENDERING_MODES = ['rgb_array', 'human', 'tiny_rgb_array', 'tiny_human', 'raw']
