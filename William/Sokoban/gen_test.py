import gym
import gym_sokoban


env = gym.make("Sokoban-v0")
env.reset()
env.render()

reward_tot = 0
done = False
while not done:
    ac = input()
    ac = ac.split()
    input_list = [int(i) for i in ac]
    state, reward, done, info = env.step(input_list, "raw")
    print(reward)
    print(info)
    reward_tot += reward
    env.render()
