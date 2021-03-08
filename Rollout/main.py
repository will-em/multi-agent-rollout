import gym
import gym_sokoban
import numpy as np


env = gym.make("Sokoban-v0")
env.reset()
env.render()

reward_tot = 0
done = False


# def base_policy(state, pos, target):


while not done:
    ac = input()
    ac = ac.split()
    input_list = [int(i) for i in ac]
    target_dict = {(2, 6): "Box 1", (6, 7): "Box 2", (6, 8)
                    : "Box 3", (3, 8): "Drop off point"}
    state, reward, done, info = env.step(input_list, "raw")
    target_list = state[1]
    for i, target in enumerate(target_list):
        print("Agent ", i+1, "s target: ", target_dict[target])
    print("Reward: ", reward)
    print(info)
    reward_tot += reward
    env.render()
