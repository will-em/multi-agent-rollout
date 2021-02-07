import numpy as np
import tensorflow as tf
from tensorflow import keras
import gym as gym
import gym_sokoban
model = keras.models.load_model("my_model_new")


def epsilon_greedy_policy(state, epsilon=0):
    if np.random.rand() < epsilon:
        return np.random.randint(2)
    else:
        Q_values = model.predict(state[np.newaxis])
        print(Q_values[0])
        return np.argmax(Q_values[0])


env = gym.make("Sokoban-v0")


def state_transform(_state):
    walls = _state[0]
    goals = _state[1]
    boxes = _state[2]
    player = _state[3]
    curr_state = walls + 2*goals + 3*boxes + 4*player
    return curr_state / 6


#seed = np.random.randint(low=0, high=1000, size=10)
for i in range(10):
    # env.seed(int(seed[i]))
    state = env.reset(render_mode='raw')
    reward_tot = 0
    for step in range(200):
        action = epsilon_greedy_policy(state_transform(state))
        state, reward, done, info = env.step(action, "raw")
        reward_tot += reward
        env.render("human")
        if done:
            break
    print(reward_tot)
