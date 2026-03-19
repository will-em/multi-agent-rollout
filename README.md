# Multiagent Rollout with Reshuffling for Warehouse Robots Path Planning

<img src="https://media.giphy.com/media/sr2ksjKHq1DnrWBwKE/giphy.gif" width="100%">

# About
This code is part of the paper [*Multiagent Rollout with Reshuffling for Warehouse Robots Path Planning*](https://arxiv.org/abs/2211.08201).

# Installation

Clone the repository:
```sh
git clone https://github.com/will-em/multi-agent-rollout.git
cd multi-agent-rollout 
```
To compile and run **Multiagent rollout**:
```sh
make && ./rollout 
```
<!-- -->
For benchmarking purposes, the **[Cooperative A*](https://www.semanticscholar.org/paper/Cooperative-Pathfinding-Silver/03ef7f3a962319a8d97cacb6afa5380948eba1be)** algorithm has also been implemented. To compile and run it:
```sh
make coop && ./coop
```
<!--Explain the output of the program-->
# How to customize scenario

If you want to modify the layout or the cost function of the environment, please see [Environment.cpp](src/Environment.cpp). Furthermore, if you want to modify the delivery points, they are defined in [Simulate.cpp](src/Simulate.cpp) and [SimulateCoop.cpp](src/SimulateCoop.cpp) for each method respectivly. Likewise, to specify the number of agents, go to file [Rollout.cpp](src/Rollout.cpp) or [Coop.cpp](src/Coop.cpp).

## Python environment

The repository now includes a Python reimplementation of the warehouse stack for reinforcement learning and rapid experimentation. `python/warehouse_environment.py` contains the multi-agent environment with boxes, drop-off points, charging stations, and per-agent battery state. `python/rollout.py` adds a battery-aware rollout policy plus an RL-friendly wrapper that exposes observations, rewards, done flags, feasible-action masks, and flattened features for learning. `scripts/train_rl.py` wires the battery-aware environment into PyTorch PPO and DQN training/evaluation, `scripts/visualize_rollout.py` can replay ASCII visualizations in the terminal or save frames to disk, and `configs/*.json` / `scripts/run_experiment_suite.py` provide reproducible experiment sets. `docs/rl_experiment_design.md` and `docs/algorithm_comparison.md` explain how to compare learned policies against rollout and random baselines and how to interpret completion rate, completion time, reward, and boxes-left metrics. Basic regression tests for the Python stack live in `tests/test_python_environment.py`.
