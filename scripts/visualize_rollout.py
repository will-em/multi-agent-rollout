from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import argparse
import time

from python import BatteryAwareRolloutPolicy, MultiAgentRolloutEnv, RandomMaskedPolicy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize rollout episodes in the terminal or save ASCII frames.")
    parser.add_argument("--policy", choices=["battery_rollout", "random_masked"], default="battery_rollout")
    parser.add_argument("--num-agents", type=int, default=2)
    parser.add_argument("--battery-capacity", type=int, default=30)
    parser.add_argument("--move-discharge", type=int, default=1)
    parser.add_argument("--idle-discharge", type=int, default=0)
    parser.add_argument("--charge-rate", type=int, default=5)
    parser.add_argument("--max-steps", type=int, default=50)
    parser.add_argument("--sleep", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--save-frames", type=Path, default=None)
    parser.add_argument("--no-clear", action="store_true")
    return parser.parse_args()


def choose_actions(env: MultiAgentRolloutEnv, policy_name: str, rollout_policy, random_policy):
    if policy_name == "battery_rollout":
        return rollout_policy.choose_actions(env.env, env.targets)
    return random_policy.choose_actions(env.env, env.targets)


def render_frame(env: MultiAgentRolloutEnv, step_idx: int, reward: float | None = None) -> str:
    batteries = ", ".join(str(value) for value in env.env.get_agent_batteries())
    header = f"step={step_idx} boxes_left={env.env.boxes_left} batteries=[{batteries}]"
    if reward is not None:
        header += f" reward={reward:.2f}"
    return header + "\n" + env.env.render_ascii()


def main() -> None:
    args = parse_args()
    env = MultiAgentRolloutEnv(
        num_agents=args.num_agents,
        battery_capacity=args.battery_capacity,
        move_discharge=args.move_discharge,
        idle_discharge=args.idle_discharge,
        charge_rate=args.charge_rate,
        seed=args.seed,
    )
    rollout_policy = BatteryAwareRolloutPolicy(seed=args.seed)
    random_policy = RandomMaskedPolicy(seed=args.seed)

    output_frames = []
    env.reset()
    output_frames.append(render_frame(env, step_idx=0))
    for step_idx in range(1, args.max_steps + 1):
        actions = choose_actions(env, args.policy, rollout_policy, random_policy)
        _, reward, done, _ = env.step(actions)
        frame = render_frame(env, step_idx=step_idx, reward=reward)
        output_frames.append(frame)
        if args.save_frames is None:
            if not args.no_clear:
                print("\033[2J\033[H", end="")
            print(frame)
            time.sleep(args.sleep)
        if done:
            break

    if args.save_frames is not None:
        args.save_frames.parent.mkdir(parents=True, exist_ok=True)
        args.save_frames.write_text("\n\n--- FRAME ---\n\n".join(output_frames) + "\n")
        print(args.save_frames)


if __name__ == "__main__":
    main()
