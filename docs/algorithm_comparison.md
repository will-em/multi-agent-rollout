# PPO、DQN、Battery-Aware Rollout 的原理与结果解读

> 下面的内容是**原理解释与结果判读指南**，不是在当前容器里实际跑出来的数值结论。

## 1. 三类方法分别在做什么

### Battery-aware rollout baseline

这是一个**人工设计的启发式策略**：

- 先根据当前目标（箱子 / 投递点 / 充电站）选择动作；
- 当电量低于阈值时，优先回到最近充电站；
- 只在 `battery_feasible_actions` 允许的动作集合中行动。

优点：

- 不需要训练；
- 一开始就稳定；
- 通常完成率较高，尤其是在低电量约束很强时。

局限：

- 难以自动学到更全局的协作；
- 目标切换和拥堵处理主要靠规则；
- 如果场景变化大，规则需要手工调。

### PPO

PPO（Proximal Policy Optimization）是**策略梯度 + value function** 的方法：

- 网络直接输出每个 agent 的动作分布；
- 采样动作后，根据回报和 advantage 更新策略；
- 通过 clip 机制限制每次更新不要太激进。

你可以把它理解成：

- baseline 是“人写规则”；
- PPO 是“让网络在 rollout 环境中不断试错，并逐渐学会更好的动作概率分布”。

常见特点：

- 训练更稳定；
- 更适合连续迭代优化策略；
- 在复杂多步决策里，通常比 DQN 更容易学到可用策略。

### DQN

DQN（Deep Q-Network）是**值函数方法**：

- 网络估计每个 joint action 的 Q 值；
- 通过 replay buffer 和 target network 做自举学习；
- 每一步选当前 Q 值最大的动作。

在这个项目里，DQN 的难点是：

- 多 agent 时 joint action 空间会快速变大；
- agent 越多，Q 值学习越难；
- 因此它在 2 agents 时可以试，但 agent 数量继续增加时通常会比 PPO 更吃力。

## 2. 一般应该怎样看比较结果

训练结束后，`comparison_metrics.json` 里会有：

- `completion_rate`
- `avg_completion_time`
- `avg_episode_reward`
- `avg_boxes_left`

### completion_rate（任务完成率）

这是最重要的指标。

- 如果 PPO / DQN 明显高于 `random_masked`，说明它确实学到了任务结构；
- 如果还明显低于 `battery_rollout`，说明训练还不够、特征不够，或 reward 设计不够好；
- 如果超过 `battery_rollout`，说明学习方法在协作或路径选择上找到了更优策略。

### avg_completion_time（平均完成时间）

这个指标反映“做完任务有多快”。

- 完成率差不多时，谁的时间更短，谁更高效；
- 如果 reward 很高但 completion time 很差，可能策略在“刷 reward”而不是快完成任务；
- 如果 rollout baseline 完成率高但时间偏慢，PPO 有机会通过学习更激进的路径分配把时间降下来。

### avg_episode_reward（平均回报）

这个指标最容易受 reward shaping 影响。

- 它适合用来观察训练趋势；
- 但最终模型优劣仍应优先看 completion rate 和 completion time；
- 如果 reward 提升、完成率却不变，说明网络可能学会了“局部最优行为”。

### avg_boxes_left（平均剩余箱子数）

这个指标适合分析失败模式。

- 数字越小越好；
- 如果完成率低且剩余箱子很多，说明策略很早就卡住；
- 如果剩余箱子很少但还是没完成，说明后期调度或回充策略存在问题。

## 3. 你大概率会看到的趋势

这些是**经验预期**，不是当前仓库已经验证的最终实验结论：

1. `random_masked` 通常最差，因为它虽然满足电量约束，但不懂任务目标。
2. `battery_rollout` 通常是一个很强的冷启动 baseline，因为它本身就编码了任务规则和充电逻辑。
3. PPO 通常最有希望逐渐接近甚至超过 rollout baseline，尤其是在 reward 设计合理、训练轮数足够时。
4. DQN 在 agent 数较少时可能有效，但当 joint action 空间增大时，往往更难训练稳定。

## 4. 建议怎样解释实验结果

### 情况 A：PPO > rollout baseline

可以解释为：

- 网络学到了比手工规则更好的全局协调；
- 电量约束不再只是“避免失败”，而被更好地纳入了全局任务规划；
- 多 agent 的目标切换和拥堵处理可能更优。

### 情况 B：PPO < rollout baseline，但 > random_masked

可以解释为：

- 强化学习已经学到了任务结构；
- 但仍未超过手工策略；
- 后续可以从更长训练、改 observation、改 reward、改网络结构入手。

### 情况 C：DQN 表现明显差于 PPO

通常是因为：

- joint action 空间太大；
- 值函数逼近难度高；
- 多 agent 环境中的非平稳性让 DQN 更难收敛。

### 情况 D：两个 RL 方法都不如 rollout baseline

这通常不代表 RL 不行，更常见原因是：

- 训练轮数还不够；
- observation 太弱；
- reward shaping 不合适；
- baseline 本身已经非常强。

## 5. 如何查看可视化结果

仓库里新增了 `scripts/visualize_rollout.py`，你可以直接在终端播放 ASCII 动画：

```bash
python scripts/visualize_rollout.py --policy battery_rollout --num-agents 2 --max-steps 50
```

如果你想把每一步保存下来，方便之后查看：

```bash
python scripts/visualize_rollout.py \
  --policy battery_rollout \
  --num-agents 2 \
  --max-steps 50 \
  --save-frames artifacts/visualization/battery_rollout_frames.txt
```

这样会把每一帧 ASCII 地图保存到文本文件中。

## 6. 如何跑可复现实验集

仓库里提供了：

- `configs/ppo_smoke.json`
- `configs/dqn_smoke.json`
- `configs/compare_both_small.json`
- `configs/repro_suite.json`

### 单个配置运行

```bash
python scripts/train_rl.py --config configs/compare_both_small.json
```

### 整套实验运行

```bash
python scripts/run_experiment_suite.py configs/repro_suite.json
```

### 结果转 markdown 表

```bash
python scripts/experiment_report.py artifacts/compare_both_small/comparison_metrics.json
```
