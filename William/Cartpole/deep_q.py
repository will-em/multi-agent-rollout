import matplotlib.pyplot as plt
from collections import deque
import gym
import tensorflow as tf
from tensorflow import keras
import numpy as np

env = gym.make("CartPole-v0")
input_shape = [4]  # env.observation_space.shape
n_outputs = 2  # env.action_space.n

model = keras.models.Sequential([
    keras.layers.Flatten(input_shape=input_shape)
    keras.layers.Dense(64, activation="elu"),
    keras.layers.Dense(64, activation="elu"),
    keras.layers.Dense(n_outputs)
])
print(env.observation_space.shape)
print(model.summary())


def epsilon_greedy_policy(state, epsilon=0):
    if np.random.rand() < epsilon:
        return np.random.randint(2)
    else:
        Q_values = model.predict(state[np.newaxis])
        return np.argmax(Q_values[0])


# Replay memory or replay buffer (reduces correlation between experiances in a training batch)

# Deque is a linked list with pointers forwards and backwards
replay_buffer = deque(maxlen=2000)


def sample_experiences(batch_size):
    indices = np.random.randint(len(replay_buffer), size=batch_size)
    batch = [replay_buffer[index] for index in indices]
    states, actions, rewards, next_states, dones = [
        np.array([experience[field_index] for experience in batch])
        for field_index in range(5)
    ]
    return states, actions, rewards, next_states, dones  # Experiences


def play_one_step(envc, state, epsilon):
    action = epsilon_greedy_policy(state, epsilon)
    next_state, reward, done, info = env.step(action)
    replay_buffer.append((state, action, reward, next_state, done))
    return next_state, reward, done, info


batch_size = 64
discount_factor = 0.95
optimizer = keras.optimizers.Adam(lr=1e-3)
loss_fn = keras.losses.mean_squared_error


def training_step(batch_size):
    experiences = sample_experiences(batch_size)
    states, actions, rewards, next_states, dones = experiences
    next_Q_values = model.predict(next_states)
    max_next_Q_values = np.max(next_Q_values, axis=1)

    target_Q_values = (rewards + (1 - dones) *
                       discount_factor * max_next_Q_values)
    target_Q_values = target_Q_values.reshape(-1, 1)

    mask = tf.one_hot(actions, n_outputs)
    with tf.GradientTape() as tape:
        all_Q_values = model(states)
        Q_values = tf.reduce_sum(all_Q_values * mask, axis=1, keepdims=True)
        loss = tf.reduce_mean(loss_fn(target_Q_values, Q_values))
    grads = tape.gradient(loss, model.trainable_variables)
    optimizer.apply_gradients(zip(grads, model.trainable_variables))


# env.seed(42)
np.random.seed(42)
tf.random.set_seed(42)

rewards = []
best_score = 0
num_of_ep = 600
for episode in range(num_of_ep):
    env.seed(episode)
    obs = env.reset()
    for step in range(200):
        epsilon = max(1 - episode / num_of_ep, 0.01)
        obs, reward, done, info = play_one_step(env, obs, epsilon)
        if done:
            break
    rewards.append(step)  # Not shown in the book
    if step > best_score:  # Not shown
        best_weights = model.get_weights()  # Not shown
        best_score = step  # Not shown
    print("\rEpisode: {}, Steps: {}, eps: {:.3f}".format(
        episode, step + 1, epsilon), end="")  # Not shown
    if episode > 100:
        training_step(batch_size)

model.set_weights(best_weights)

plt.figure(figsize=(8, 4))
plt.plot(rewards)
plt.xlabel("Episode", fontsize=14)
plt.ylabel("Sum of rewards", fontsize=14)
plt.show()

model.save("my_model")
