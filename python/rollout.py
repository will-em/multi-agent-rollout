from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Callable, Dict, List, Sequence

from python.pathfinding import manhattan_distance
from python.warehouse_environment import Action, WarehouseEnvironment

MAX_NUMBER_OF_SHUFFLES = 10_000
RESHUFFLING_THRESHOLD = 10_000.0


@dataclass
class RolloutTransition:
    observation: Dict[str, object]
    actions: List[int]
    reward: float
    next_observation: Dict[str, object]
    done: bool
    info: Dict[str, object]


class BatteryAwareRolloutPolicy:
    def __init__(self, reshuffling_threshold: float = RESHUFFLING_THRESHOLD, seed: int | None = None) -> None:
        self.reshuffling_threshold = reshuffling_threshold
        self.rng = random.Random(seed)

    def choose_action(
        self,
        env: WarehouseEnvironment,
        agent_idx: int,
        targets: Sequence[tuple[int, int] | None],
    ) -> int:
        agent = env.agent_states[agent_idx]
        target = targets[agent_idx]

        if agent.battery <= env.low_battery_threshold:
            target = env.nearest_charging_station(agent.position)

        feasible_actions = env.battery_feasible_actions(agent_idx, target)
        if target is None:
            return int(Action.STAY)

        path = env.shortest_actions_to_target(agent_idx, target)
        preferred = int(path[0]) if path else int(Action.STAY)
        if preferred in feasible_actions:
            return preferred

        best_action = min(
            feasible_actions,
            key=lambda action: self._post_action_distance(env, agent_idx, action, target),
        )
        return best_action

    def _post_action_distance(
        self,
        env: WarehouseEnvironment,
        agent_idx: int,
        action: int,
        target: tuple[int, int],
    ) -> int:
        agent = env.agent_states[agent_idx]
        candidate = env._next_position(agent.position, action)
        if not env._valid_destination(candidate, target):
            candidate = agent.position
        return manhattan_distance(candidate, target)

    def choose_actions(
        self,
        env: WarehouseEnvironment,
        targets: Sequence[tuple[int, int] | None],
        agent_order: Sequence[int] | None = None,
    ) -> List[int]:
        order = list(agent_order) if agent_order is not None else list(range(len(env.agent_states)))
        actions = [int(Action.STAY)] * len(env.agent_states)
        for agent_idx in order:
            actions[agent_idx] = self.choose_action(env, agent_idx, targets)
        return actions


class RandomMaskedPolicy:
    def __init__(self, seed: int | None = None) -> None:
        self.rng = random.Random(seed)

    def choose_actions(
        self,
        env: WarehouseEnvironment,
        targets: Sequence[tuple[int, int] | None],
        agent_order: Sequence[int] | None = None,
    ) -> List[int]:
        order = list(agent_order) if agent_order is not None else list(range(len(env.agent_states)))
        actions = [int(Action.STAY)] * len(env.agent_states)
        for agent_idx in order:
            feasible = env.battery_feasible_actions(agent_idx, targets[agent_idx])
            actions[agent_idx] = self.rng.choice(feasible)
        return actions


class MultiAgentRolloutEnv:
    """RL-friendly wrapper that exposes battery-constrained rollout transitions."""

    def __init__(
        self,
        num_agents: int,
        battery_capacity: int = 100,
        move_discharge: int = 1,
        idle_discharge: int = 0,
        charge_rate: int = 10,
        seed: int | None = None,
    ) -> None:
        self.num_agents = num_agents
        self.seed = seed
        self.battery_capacity = battery_capacity
        self.move_discharge = move_discharge
        self.idle_discharge = idle_discharge
        self.charge_rate = charge_rate
        self.low_battery_threshold = 15
        self.charge_safety_margin = 3
        self.policy = BatteryAwareRolloutPolicy(seed=seed)
        self._build_env()

    def _build_env(self) -> None:
        self.env = WarehouseEnvironment(
            agent_count=self.num_agents,
            battery_capacity=self.battery_capacity,
            move_discharge=self.move_discharge,
            idle_discharge=self.idle_discharge,
            charge_rate=self.charge_rate,
            low_battery_threshold=self.low_battery_threshold,
            charge_safety_margin=self.charge_safety_margin,
        )
        self.targets = self.env.assign_initial_targets()

    def reset(self) -> Dict[str, object]:
        self._build_env()
        return self.env.get_observation()

    def step(self, actions: Sequence[int]) -> tuple[Dict[str, object], float, bool, Dict[str, object]]:
        before_values = self.env.get_agent_values()
        cost, info = self.env.step(actions, self.targets)
        self.env.update_targets(before_values, self.targets)
        observation = self.env.get_observation()
        reward = -cost
        done = self.env.is_done()
        return observation, reward, done, {"step_info": vars(info), "targets": list(self.targets)}

    def rollout_step(self) -> tuple[Dict[str, object], float, bool, Dict[str, object]]:
        actions = self.policy.choose_actions(self.env, self.targets)
        return self.step(actions)

    def sample_action_mask(self) -> List[List[int]]:
        return [self.env.battery_feasible_actions(idx, self.targets[idx]) for idx in range(self.num_agents)]

    def flatten_observation(self, observation: Dict[str, object] | None = None) -> List[float]:
        obs = observation or self.env.get_observation()
        features: List[float] = [float(obs["step_count"]), float(obs["boxes_left"])]
        positions = obs["agent_positions"]
        batteries = obs["agent_batteries"]
        carrying = obs["agent_carrying"]
        targets = obs["agent_targets"]
        for idx in range(self.num_agents):
            row, col = positions[idx]
            features.extend([row / self.env.height, col / self.env.width])
            features.append(batteries[idx] / max(1.0, float(self.battery_capacity)))
            features.append(1.0 if carrying[idx] else 0.0)
            target = targets[idx] if targets[idx] is not None else positions[idx]
            features.extend([target[0] / self.env.height, target[1] / self.env.width])
        for charger in sorted(self.env.charging_stations):
            features.extend([charger[0] / self.env.height, charger[1] / self.env.width])
        return features

    @property
    def observation_dim(self) -> int:
        return len(self.flatten_observation())

    @property
    def action_dim_per_agent(self) -> int:
        return len(Action)

    @property
    def joint_action_dim(self) -> int:
        return self.num_agents * self.action_dim_per_agent


def simulate_rollout(
    num_agents: int,
    display_environment: bool = False,
    max_steps: int = 2000,
    seed: int | None = None,
    battery_capacity: int = 100,
    move_discharge: int = 1,
    idle_discharge: int = 0,
    charge_rate: int = 10,
) -> List[RolloutTransition]:
    env = MultiAgentRolloutEnv(
        num_agents=num_agents,
        battery_capacity=battery_capacity,
        move_discharge=move_discharge,
        idle_discharge=idle_discharge,
        charge_rate=charge_rate,
        seed=seed,
    )
    transitions: List[RolloutTransition] = []

    for _ in range(max_steps):
        observation = env.env.get_observation()
        actions = env.policy.choose_actions(env.env, env.targets)
        next_observation, reward, done, info = env.step(actions)
        transitions.append(
            RolloutTransition(
                observation=observation,
                actions=list(actions),
                reward=reward,
                next_observation=next_observation,
                done=done,
                info=info,
            )
        )
        if display_environment:
            print(env.env.render_ascii())
        if done:
            break

    return transitions


def run_episode(
    env: MultiAgentRolloutEnv,
    action_fn: Callable[[MultiAgentRolloutEnv], Sequence[int]],
    max_steps: int,
) -> Dict[str, float]:
    observation = env.reset()
    total_reward = 0.0
    for step in range(max_steps):
        actions = list(action_fn(env))
        observation, reward, done, _ = env.step(actions)
        total_reward += reward
        if done:
            return {
                "completed": 1.0,
                "completion_time": float(step + 1),
                "episode_reward": total_reward,
                "boxes_left": float(observation["boxes_left"]),
            }
    return {
        "completed": 0.0,
        "completion_time": float(max_steps),
        "episode_reward": total_reward,
        "boxes_left": float(observation["boxes_left"]),
    }


__all__ = [
    "BatteryAwareRolloutPolicy",
    "MAX_NUMBER_OF_SHUFFLES",
    "MultiAgentRolloutEnv",
    "RESHUFFLING_THRESHOLD",
    "RolloutTransition",
    "RandomMaskedPolicy",
    "run_episode",
    "simulate_rollout",
]
