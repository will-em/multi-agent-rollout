from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a markdown experiment summary from comparison_metrics.json.")
    parser.add_argument("metrics", type=Path, help="Path to comparison_metrics.json")
    parser.add_argument("--output", type=Path, default=Path("artifacts/rl/experiment_report.md"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    metrics = json.loads(args.metrics.read_text())
    lines = [
        "# RL Experiment Summary",
        "",
        "| Method | Completion Rate | Avg Completion Time | Avg Episode Reward | Avg Boxes Left |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for method, values in metrics.items():
        lines.append(
            f"| {method} | {values['completion_rate']:.3f} | {values['avg_completion_time']:.2f} | "
            f"{values['avg_episode_reward']:.2f} | {values['avg_boxes_left']:.2f} |"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
