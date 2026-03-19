from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import argparse
import json
import random
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Sequence, Tuple

from python import MultiAgentRolloutEnv, RandomMaskedPolicy, run_episode

try:
    import torch
    from torch import nn
    from torch.distributions import Categorical
except ModuleNotFoundError as exc:  # pragma: no cover - environment specific
    raise SystemExit(
        "PyTorch is required for scripts/train_rl.py. Install torch first, then rerun the script."
    ) from exc


@dataclass
class Transition:
    observation: torch.Tensor
    action: torch.Tensor
    log_prob: torch.Tensor
    reward: float
    done: float
    value: torch.Tensor
    mask: torch.Tensor


class PolicyNetwork(nn.Module):
    def __init__(self, observation_dim: int, num_agents: int, action_dim: int, hidden_dim: int = 256) -> None:
        super().__init__()
        self.num_agents = num_agents
        self.action_dim = action_dim
        self.backbone = nn.Sequential(
            nn.Linear(observation_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.policy_head = nn.Linear(hidden_dim, num_agents * action_dim)
        self.value_head = nn.Linear(hidden_dim, 1)

    def forward(self, obs: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        hidden = self.backbone(obs)
        logits = self.policy_head(hidden).view(-1, self.num_agents, self.action_dim)
        value = self.value_head(hidden).squeeze(-1)
        return logits, value


class QNetwork(nn.Module):
    def __init__(self, observation_dim: int, output_dim: int, hidden_dim: int = 256) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(observation_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)


class ReplayBuffer:
    def __init__(self, capacity: int) -> None:
        self.buffer: Deque[Tuple[torch.Tensor, torch.Tensor, float, torch.Tensor, float]] = deque(maxlen=capacity)

    def push(
        self,
        observation: torch.Tensor,
        action: torch.Tensor,
        reward: float,
        next_observation: torch.Tensor,
        done: float,
    ) -> None:
        self.buffer.append((observation, action, reward, next_observation, done))

    def sample(self, batch_size: int):
        batch = random.sample(self.buffer, batch_size)
        obs, action, reward, next_obs, done = zip(*batch)
        return (
            torch.stack(list(obs)),
            torch.stack(list(action)),
            torch.tensor(reward, dtype=torch.float32),
            torch.stack(list(next_obs)),
            torch.tensor(done, dtype=torch.float32),
        )

    def __len__(self) -> int:
        return len(self.buffer)


def set_seed(seed: int) -> None:
    random.seed(seed)
    torch.manual_seed(seed)


def observation_tensor(env: MultiAgentRolloutEnv) -> torch.Tensor:
    return torch.tensor(env.flatten_observation(), dtype=torch.float32)


def action_mask_tensor(env: MultiAgentRolloutEnv) -> torch.Tensor:
    mask = torch.zeros((env.num_agents, env.action_dim_per_agent), dtype=torch.bool)
    for idx, feasible in enumerate(env.sample_action_mask()):
        for action in feasible:
            mask[idx, action] = True
    return mask


def masked_categorical(logits: torch.Tensor, mask: torch.Tensor) -> Categorical:
    masked_logits = logits.masked_fill(~mask, -1e9)
    return Categorical(logits=masked_logits)



def expand_joint_action(encoded: int, num_agents: int, action_dim: int) -> List[int]:
    actions = [0] * num_agents
    for idx in range(num_agents - 1, -1, -1):
        actions[idx] = encoded % action_dim
        encoded //= action_dim
    return actions


def rollout_baseline_actions(env: MultiAgentRolloutEnv) -> Sequence[int]:
    return env.policy.choose_actions(env.env, env.targets)


def random_baseline_actions(random_policy: RandomMaskedPolicy, env: MultiAgentRolloutEnv) -> Sequence[int]:
    return random_policy.choose_actions(env.env, env.targets)


def evaluate_baselines(config: argparse.Namespace, checkpoints: Dict[str, Path] | None = None) -> Dict[str, Dict[str, float]]:
    baselines: Dict[str, Dict[str, float]] = {}
    random_policy = RandomMaskedPolicy(seed=config.seed)
    baseline_specs = {
        "battery_rollout": lambda env: rollout_baseline_actions(env),
        "random_masked": lambda env: random_baseline_actions(random_policy, env),
    }
    for name, action_fn in baseline_specs.items():
        metrics = evaluate_policy(config, action_fn)
        baselines[name] = metrics

    if checkpoints:
        for algo, path in checkpoints.items():
            if path.exists():
                metrics = evaluate_saved_policy(config, algo, path)
                baselines[algo] = metrics
    return baselines


def evaluate_policy(config: argparse.Namespace, action_fn, episodes: int | None = None) -> Dict[str, float]:
    episode_count = episodes or config.eval_episodes
    results = []
    for episode_idx in range(episode_count):
        env = MultiAgentRolloutEnv(
            num_agents=config.num_agents,
            battery_capacity=config.battery_capacity,
            move_discharge=config.move_discharge,
            idle_discharge=config.idle_discharge,
            charge_rate=config.charge_rate,
            seed=config.seed + episode_idx,
        )
        results.append(run_episode(env, action_fn, config.max_steps))
    return aggregate_metrics(results)


def aggregate_metrics(results: List[Dict[str, float]]) -> Dict[str, float]:
    count = max(1, len(results))
    return {
        "completion_rate": sum(item["completed"] for item in results) / count,
        "avg_completion_time": sum(item["completion_time"] for item in results) / count,
        "avg_episode_reward": sum(item["episode_reward"] for item in results) / count,
        "avg_boxes_left": sum(item["boxes_left"] for item in results) / count,
    }


def compute_returns_and_advantages(
    transitions: List[Transition],
    last_value: torch.Tensor,
    gamma: float,
    gae_lambda: float,
) -> Tuple[torch.Tensor, torch.Tensor]:
    advantages = []
    gae = 0.0
    values = [transition.value.item() for transition in transitions] + [last_value.item()]
    for index in reversed(range(len(transitions))):
        delta = transitions[index].reward + gamma * values[index + 1] * (1.0 - transitions[index].done) - values[index]
        gae = delta + gamma * gae_lambda * (1.0 - transitions[index].done) * gae
        advantages.insert(0, gae)
    advantages_tensor = torch.tensor(advantages, dtype=torch.float32)
    returns_tensor = advantages_tensor + torch.tensor(values[:-1], dtype=torch.float32)
    return returns_tensor, advantages_tensor


def train_ppo(config: argparse.Namespace, output_dir: Path) -> Path:
    env = MultiAgentRolloutEnv(
        num_agents=config.num_agents,
        battery_capacity=config.battery_capacity,
        move_discharge=config.move_discharge,
        idle_discharge=config.idle_discharge,
        charge_rate=config.charge_rate,
        seed=config.seed,
    )
    model = PolicyNetwork(env.observation_dim, env.num_agents, env.action_dim_per_agent, config.hidden_dim)
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate)
    checkpoint_path = output_dir / "ppo_policy.pt"

    for episode in range(config.episodes):
        transitions: List[Transition] = []
        env.reset()
        for _ in range(config.max_steps):
            obs = observation_tensor(env)
            mask = action_mask_tensor(env)
            logits, value = model(obs.unsqueeze(0))
            logits = logits.squeeze(0)
            value = value.squeeze(0)
            sampled_actions = []
            log_probs = []
            for agent_idx in range(env.num_agents):
                distribution = masked_categorical(logits[agent_idx], mask[agent_idx])
                action = distribution.sample()
                sampled_actions.append(int(action.item()))
                log_probs.append(distribution.log_prob(action))
            _, reward, done, _ = env.step(sampled_actions)
            transitions.append(
                Transition(
                    observation=obs,
                    action=torch.tensor(sampled_actions, dtype=torch.long),
                    log_prob=torch.stack(log_probs).sum(),
                    reward=reward,
                    done=float(done),
                    value=value,
                    mask=mask,
                )
            )
            if done:
                break

        next_obs = observation_tensor(env)
        _, next_value = model(next_obs.unsqueeze(0))
        returns, advantages = compute_returns_and_advantages(
            transitions,
            next_value.squeeze(0).detach(),
            config.gamma,
            config.gae_lambda,
        )
        if len(transitions) == 0:
            continue
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        observations = torch.stack([transition.observation for transition in transitions])
        actions = torch.stack([transition.action for transition in transitions])
        old_log_probs = torch.stack([transition.log_prob for transition in transitions]).detach()
        masks = torch.stack([transition.mask for transition in transitions])

        for _ in range(config.ppo_epochs):
            logits, values = model(observations)
            per_step_log_probs = []
            entropies = []
            for step_idx in range(logits.shape[0]):
                step_log_prob = 0.0
                step_entropy = 0.0
                for agent_idx in range(env.num_agents):
                    distribution = masked_categorical(logits[step_idx, agent_idx], masks[step_idx, agent_idx])
                    step_log_prob = step_log_prob + distribution.log_prob(actions[step_idx, agent_idx])
                    step_entropy = step_entropy + distribution.entropy()
                per_step_log_probs.append(step_log_prob)
                entropies.append(step_entropy)
            new_log_probs = torch.stack(per_step_log_probs)
            entropy = torch.stack(entropies).mean()
            ratio = (new_log_probs - old_log_probs).exp()
            surrogate1 = ratio * advantages
            surrogate2 = torch.clamp(ratio, 1.0 - config.clip_range, 1.0 + config.clip_range) * advantages
            policy_loss = -torch.min(surrogate1, surrogate2).mean()
            value_loss = torch.nn.functional.mse_loss(values, returns)
            loss = policy_loss + config.value_coef * value_loss - config.entropy_coef * entropy
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), config.max_grad_norm)
            optimizer.step()

        if (episode + 1) % config.log_interval == 0:
            print(f"[PPO] episode={episode + 1}/{config.episodes} transitions={len(transitions)}")

    torch.save({"model_state": model.state_dict(), "config": vars(config)}, checkpoint_path)
    return checkpoint_path


def train_dqn(config: argparse.Namespace, output_dir: Path) -> Path:
    env = MultiAgentRolloutEnv(
        num_agents=config.num_agents,
        battery_capacity=config.battery_capacity,
        move_discharge=config.move_discharge,
        idle_discharge=config.idle_discharge,
        charge_rate=config.charge_rate,
        seed=config.seed,
    )
    joint_action_dim = env.action_dim_per_agent ** env.num_agents
    q_network = QNetwork(env.observation_dim, joint_action_dim, config.hidden_dim)
    target_network = QNetwork(env.observation_dim, joint_action_dim, config.hidden_dim)
    target_network.load_state_dict(q_network.state_dict())
    optimizer = torch.optim.Adam(q_network.parameters(), lr=config.learning_rate)
    replay_buffer = ReplayBuffer(config.replay_capacity)
    checkpoint_path = output_dir / "dqn_policy.pt"
    epsilon = config.epsilon_start

    for episode in range(config.episodes):
        env.reset()
        for step in range(config.max_steps):
            obs = observation_tensor(env)
            feasible_actions = env.sample_action_mask()
            all_joint_actions = []
            for encoded in range(joint_action_dim):
                joint_action = expand_joint_action(encoded, env.num_agents, env.action_dim_per_agent)
                if all(joint_action[idx] in feasible_actions[idx] for idx in range(env.num_agents)):
                    all_joint_actions.append(encoded)
            if not all_joint_actions:
                all_joint_actions = [0]

            if random.random() < epsilon:
                encoded_action = random.choice(all_joint_actions)
            else:
                with torch.no_grad():
                    q_values = q_network(obs.unsqueeze(0)).squeeze(0)
                    invalid_mask = torch.ones_like(q_values, dtype=torch.bool)
                    invalid_mask[all_joint_actions] = False
                    q_values = q_values.masked_fill(invalid_mask, -1e9)
                    encoded_action = int(torch.argmax(q_values).item())

            actions = expand_joint_action(encoded_action, env.num_agents, env.action_dim_per_agent)
            next_observation, reward, done, _ = env.step(actions)
            next_obs = torch.tensor(env.flatten_observation(next_observation), dtype=torch.float32)
            replay_buffer.push(obs, torch.tensor(encoded_action, dtype=torch.long), reward, next_obs, float(done))

            if len(replay_buffer) >= config.batch_size:
                batch_obs, batch_action, batch_reward, batch_next_obs, batch_done = replay_buffer.sample(config.batch_size)
                current_q = q_network(batch_obs).gather(1, batch_action.unsqueeze(1)).squeeze(1)
                with torch.no_grad():
                    next_q = target_network(batch_next_obs).max(dim=1).values
                    target_q = batch_reward + config.gamma * next_q * (1.0 - batch_done)
                loss = torch.nn.functional.mse_loss(current_q, target_q)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(q_network.parameters(), config.max_grad_norm)
                optimizer.step()

            if step % config.target_update_interval == 0:
                target_network.load_state_dict(q_network.state_dict())
            if done:
                break

        epsilon = max(config.epsilon_end, epsilon * config.epsilon_decay)
        if (episode + 1) % config.log_interval == 0:
            print(f"[DQN] episode={episode + 1}/{config.episodes} epsilon={epsilon:.4f}")

    torch.save({"model_state": q_network.state_dict(), "config": vars(config)}, checkpoint_path)
    return checkpoint_path


def load_policy_network(checkpoint_path: Path, observation_dim: int, num_agents: int, action_dim: int, hidden_dim: int):
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    model = PolicyNetwork(observation_dim, num_agents, action_dim, hidden_dim)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()
    return model


def load_q_network(checkpoint_path: Path, observation_dim: int, output_dim: int, hidden_dim: int):
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    model = QNetwork(observation_dim, output_dim, hidden_dim)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()
    return model


def evaluate_saved_policy(config: argparse.Namespace, algo: str, checkpoint_path: Path) -> Dict[str, float]:
    env = MultiAgentRolloutEnv(
        num_agents=config.num_agents,
        battery_capacity=config.battery_capacity,
        move_discharge=config.move_discharge,
        idle_discharge=config.idle_discharge,
        charge_rate=config.charge_rate,
        seed=config.seed,
    )
    if algo == "ppo":
        model = load_policy_network(
            checkpoint_path,
            env.observation_dim,
            env.num_agents,
            env.action_dim_per_agent,
            config.hidden_dim,
        )

        def action_fn(runtime_env: MultiAgentRolloutEnv):
            obs = torch.tensor(runtime_env.flatten_observation(), dtype=torch.float32)
            mask = action_mask_tensor(runtime_env)
            with torch.no_grad():
                logits, _ = model(obs.unsqueeze(0))
            actions = []
            for agent_idx in range(runtime_env.num_agents):
                distribution = masked_categorical(logits[0, agent_idx], mask[agent_idx])
                actions.append(int(torch.argmax(distribution.probs).item()))
            return actions

        return evaluate_policy(config, action_fn)

    if algo == "dqn":
        model = load_q_network(
            checkpoint_path,
            env.observation_dim,
            env.action_dim_per_agent ** env.num_agents,
            config.hidden_dim,
        )

        def action_fn(runtime_env: MultiAgentRolloutEnv):
            obs = torch.tensor(runtime_env.flatten_observation(), dtype=torch.float32)
            feasible = runtime_env.sample_action_mask()
            valid_joint_actions = []
            for encoded in range(runtime_env.action_dim_per_agent ** runtime_env.num_agents):
                decoded = expand_joint_action(encoded, runtime_env.num_agents, runtime_env.action_dim_per_agent)
                if all(decoded[idx] in feasible[idx] for idx in range(runtime_env.num_agents)):
                    valid_joint_actions.append(encoded)
            with torch.no_grad():
                q_values = model(obs.unsqueeze(0)).squeeze(0)
            invalid_mask = torch.ones_like(q_values, dtype=torch.bool)
            invalid_mask[valid_joint_actions] = False
            q_values = q_values.masked_fill(invalid_mask, -1e9)
            return expand_joint_action(int(torch.argmax(q_values).item()), runtime_env.num_agents, runtime_env.action_dim_per_agent)

        return evaluate_policy(config, action_fn)

    raise ValueError(f"Unsupported algorithm: {algo}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train PPO or DQN on the battery-aware warehouse rollout environment.")
    parser.add_argument("--config", type=Path, default=None, help="Optional JSON config file whose values are used as defaults.")
    parser.add_argument("--algo", choices=["ppo", "dqn", "both"], default="both")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--eval-episodes", type=int, default=5)
    parser.add_argument("--max-steps", type=int, default=200)
    parser.add_argument("--num-agents", type=int, default=2)
    parser.add_argument("--battery-capacity", type=int, default=30)
    parser.add_argument("--move-discharge", type=int, default=1)
    parser.add_argument("--idle-discharge", type=int, default=0)
    parser.add_argument("--charge-rate", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip-range", type=float, default=0.2)
    parser.add_argument("--value-coef", type=float, default=0.5)
    parser.add_argument("--entropy-coef", type=float, default=0.01)
    parser.add_argument("--ppo-epochs", type=int, default=4)
    parser.add_argument("--epsilon-start", type=float, default=1.0)
    parser.add_argument("--epsilon-end", type=float, default=0.05)
    parser.add_argument("--epsilon-decay", type=float, default=0.95)
    parser.add_argument("--target-update-interval", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--replay-capacity", type=int, default=1000)
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--max-grad-norm", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--log-interval", type=int, default=5)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/rl"))
    return parser


def parse_args() -> argparse.Namespace:
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument("--config", type=Path, default=None)
    bootstrap_args, remaining = bootstrap.parse_known_args()

    parser = build_parser()
    if bootstrap_args.config is not None:
        config_defaults = json.loads(bootstrap_args.config.read_text())
        parser.set_defaults(**config_defaults)
    return parser.parse_args(remaining)


def main() -> None:
    config = parse_args()
    set_seed(config.seed)
    config.output_dir.mkdir(parents=True, exist_ok=True)
    checkpoints: Dict[str, Path] = {}

    if config.algo in {"ppo", "both"}:
        checkpoints["ppo"] = train_ppo(config, config.output_dir)
    if config.algo in {"dqn", "both"}:
        checkpoints["dqn"] = train_dqn(config, config.output_dir)

    comparison = evaluate_baselines(config, checkpoints)
    output_path = config.output_dir / "comparison_metrics.json"
    output_path.write_text(json.dumps(comparison, indent=2, sort_keys=True))
    print(json.dumps(comparison, indent=2, sort_keys=True))
    print(f"Saved metrics to {output_path}")


if __name__ == "__main__":
    main()
