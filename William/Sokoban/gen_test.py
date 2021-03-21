import gym
import gym_sokoban
import copy


env = gym.make("Sokoban-v0")
test = env.reset(render_mode="raw")
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
    if n == 3:
        cached_state = copy.deepcopy(state) + [n]
        print("SPARAD STATE:")
    target_list = state[1]
    print("Reward: ", reward)
    print(state[0])
    print(target_list)
    reward_tot += reward
    env.render()
    n += 1

test = env.reset(render_mode="raw", cached_state=cached_state)
env.render()
print(test[0], test[1])
input()
