# Battery-Aware Rollout + PPO/DQN Experiment Design

## Goals

This document defines how to compare learning-based methods against the existing battery-aware rollout baseline in the Python warehouse environment.

### Questions to answer

1. Does PPO improve task completion rate compared with a random masked baseline?
2. Can DQN reduce average completion time after training?
3. How close can learned policies get to the handcrafted battery-aware rollout baseline?
4. Does explicitly modelling battery constraints reduce failure cases caused by energy depletion?

## Methods under comparison

- **Battery-aware rollout baseline**: the handcrafted policy from `python/rollout.py`.
- **Random masked baseline**: selects uniformly from battery-feasible actions.
- **PPO**: centralized actor-critic policy implemented in `scripts/train_rl.py`.
- **DQN**: centralized joint-action value learner implemented in `scripts/train_rl.py`.

## Metrics

For every method, record the following episode-level metrics:

- **Task completion rate**: fraction of episodes that deliver all boxes within `max_steps`.
- **Completion time**: number of environment steps used in successful episodes; for unsuccessful episodes use `max_steps`.
- **Episode reward**: total reward accumulated across the episode.
- **Boxes left**: remaining undelivered boxes at episode end.

These metrics are produced automatically into `comparison_metrics.json` by `scripts/train_rl.py`, then converted into a markdown table by `scripts/experiment_report.py`.

## Recommended experiment matrix

| Setting | Values |
| --- | --- |
| Number of agents | 2, 4 |
| Battery capacity | 20, 30 |
| Charge rate | 5, 10 |
| Seeds | 7, 11, 23 |
| Episodes | 100+ for quick smoke, 1000+ for serious comparison |
| Eval episodes | 20–100 |

## Example commands

### Train and compare PPO + DQN

```bash
python scripts/train_rl.py \
  --algo both \
  --episodes 200 \
  --eval-episodes 20 \
  --num-agents 2 \
  --battery-capacity 30 \
  --charge-rate 5 \
  --output-dir artifacts/rl_run_01
```

### Turn the JSON metrics into a markdown report

```bash
python scripts/experiment_report.py artifacts/rl_run_01/comparison_metrics.json \
  --output artifacts/rl_run_01/experiment_report.md
```

## Reading the results

A practical interpretation template:

- If **completion rate** improves over `random_masked`, the learner is using battery-aware constraints productively.
- If **avg completion time** approaches or beats `battery_rollout`, the learned policy is competitive with the handcrafted baseline.
- If **avg boxes left** remains high, training is likely unstable or the horizon is too short.
- If **episode reward** rises while completion rate does not, inspect reward shaping and low-battery penalties.

## Suggested report structure

1. Environment and training setup.
2. Hyperparameters for PPO and DQN.
3. Comparison table for completion rate / completion time / reward / boxes left.
4. Failure case analysis (collisions, deadlocks, low-battery stalls).
5. Conclusion: whether RL matches or surpasses the rollout baseline.
