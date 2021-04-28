import gym
import gym_sokoban
import copy


env = gym.make("Sokoban-v0")
test = env.reset(render_mode="raw", num_of_agents=5)
print(test)
env.render()
reward_tot = 0
done = False
n = 1
cached_state = []
while not done:
    ac = input()
    ac = ac.split()
    input_list = [int(i) for i in ac]
    state, reward, done, info = env.step(input_list, "raw")
    print("Reward: ", reward)
    #print(state[0])
    reward_tot += reward
    env.render()
    n += 1

input()