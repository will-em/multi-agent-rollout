import numpy as np
import tensorflow as tf
from tensorflow import keras
import gym as gym
model = keras.models.load_model("my_model")


def epsilon_greedy_policy(state, epsilon=0):
    if np.random.rand() < epsilon:
        return np.random.randint(2)
    else:
        Q_values = model.predict(state[np.newaxis])
        return np.argmax(Q_values[0])


env = gym.make("CartPole-v0")

seed = np.random.randint(low=0, high=1000, size=10)
for i in range(10):
    env.seed(int(seed[i]))
    state = env.reset()
    reward_tot = 0
    for step in range(200):
        action = epsilon_greedy_policy(state)
        state, reward, done, info = env.step(action)
        reward_tot += reward
        env.render()
        if done:
            break
    print(reward_tot)
    