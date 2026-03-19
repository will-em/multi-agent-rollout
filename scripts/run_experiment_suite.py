from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any, Dict, Iterable, List


SCRIPT_PATH = Path("scripts/train_rl.py")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a reproducible suite of RL experiments from JSON configs.")
    parser.add_argument("suite", type=Path, help="Path to a suite JSON file")
    parser.add_argument("--python", default="python", help="Python executable to use")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without executing them")
    return parser.parse_args()


def iter_cli_args(config: Dict[str, Any]) -> Iterable[str]:
    for key, value in config.items():
        flag = f"--{key.replace('_', '-')}"
        if isinstance(value, bool):
            if value:
                yield flag
            continue
        yield flag
        yield str(value)


def main() -> None:
    args = parse_args()
    suite = json.loads(args.suite.read_text())
    experiments: List[Dict[str, Any]] = suite["experiments"]
    for experiment in experiments:
        command = [args.python, str(SCRIPT_PATH), "--config", str(experiment["config"])]
        if "output_dir" in experiment:
            command.extend(["--output-dir", str(experiment["output_dir"])])
        if "overrides" in experiment:
            command.extend(iter_cli_args(experiment["overrides"]))
        print(" ".join(command))
        if not args.dry_run:
            subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
