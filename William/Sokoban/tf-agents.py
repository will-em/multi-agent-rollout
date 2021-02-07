
import tensorflow as tf
import numpy as np

from tf_agents.environments import py_environment
from tf_agents.environments import tf_environment
from tf_agents.environments import tf_py_environment
from tf_agents.environments import utils
from tf_agents.specs import array_spec
from tf_agents.environments import wrappers
from tf_agents.environments import suite_gym
from tf_agents.trajectories import time_step as ts

env = suite_gym.load('CartPole-v0')

tf_env = tf_py_environment.TFPyEnvironment(env)
# reset() creates the initial time_step after resetting the environment.
time_step = tf_env.reset()
num_steps = 3
transitions = []
reward = 0
for i in range(num_steps):
    action = tf.constant([i % 2])
    # applies the action and returns the new TimeStep.
    next_time_step = tf_env.step(action)
    transitions.append([time_step, action, next_time_step])
    reward += next_time_step.reward
    time_step = next_time_step

np_transitions = tf.nest.map_structure(lambda x: x.numpy(), transitions)
print('\n'.join(map(str, np_transitions)))
print('Total reward:', reward.numpy())
