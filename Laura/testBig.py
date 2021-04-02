import gym
import gym_sokoban
import numpy as np

num_of_agents = 4
env = gym.make("Sokoban-v0")
state = env.reset(render_mode="raw", num_of_agents=num_of_agents)
env.render()

input()