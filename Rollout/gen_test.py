import gym
import gym_sokoban
import copy
import tensorflow as tf
import numpy as np


env = gym.make("Sokoban-v0")
state = env.reset(render_mode="raw", num_of_agents=8)
env.render()
reward_tot = 0
done = False
n = 1
cached_state = []
new_model = tf.keras.models.load_model('./test_model')

while not done:
    input()
    mat = state[0]
    targets = state[1]
    new_mat = mat.flatten()
    for target in targets:
        new_mat = np.append(new_mat, target[0])
        new_mat = np.append(new_mat, target[1])

    prediction = new_model.predict(np.array([new_mat]))
    action_list = []
    for i in range(8):
        inner_array = prediction[0][5*i:(5*(i+1))]
        action_list.append(inner_array.argmax())
    print(action_list)
    state, reward, done, info = env.step(action_list, "raw")
    print("Reward: ", reward)
    reward_tot += reward
    env.render()
    n += 1
