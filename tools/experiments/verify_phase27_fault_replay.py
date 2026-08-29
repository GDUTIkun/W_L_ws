#!/usr/bin/env python3
"""Verify Phase 27 fault/reset, fresh replay and output non-overwrite contracts."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
IGNORE = {"episode", "core_step_ns", "phase27_solve_s", "phase27_wbc_total_s"}
FAULTS = ("nmpc_solver_failure", "nmpc_late", "nmpc_stale", "nmpc_nonfinite")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def normalized(row: dict[str, str], ignore: set[str]) -> dict[str, str]:
    return {key: value for key, value in row.items() if key not in ignore}


def exact_episodes(data: list[dict[str, str]], ignore: set[str]) -> bool:
    episodes = sorted({row["episode"] for row in data})
    sequences = [
        [normalized(row, ignore) for row in data if row["episode"] == episode]
        for episode in episodes
    ]
    return len(sequences) == 2 and sequences[0] == sequences[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--primary", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config = json.loads(args.config.read_text(encoding="utf-8"))
    equilibrium = ",".join(format(value, ".17g") for value in config["equilibrium"])
    torque = ",".join(format(value, ".17g") for value in config["torque_limit_nm"])
    fault_results = []
    for fault in FAULTS:
        control = output / f"fault_{fault}_control.csv"
        plant = output / f"fault_{fault}_plant.csv"
        command = [
            str(args.runner.resolve()), "--model", str(ROOT / config["scene"]),
            "--control-output", str(control), "--plant-output", str(plant),
            "--scenario", fault, "--controller-mode", "phase27_minimal_nmpc",
            "--phase27-reference-profile", "static", "--fault-tick", "2",
            "--ticks", "8", "--episodes", "2", "--equilibrium", equilibrium,
            "--torque-limit", torque,
        ]
        subprocess.run(command, check=True)
        control_rows = rows(control)
        plant_rows = rows(plant)
        grouped = {
            episode: [row for row in control_rows if row["episode"] == episode]
            for episode in ("0", "1")
        }
        checks = {
            "row_count": len(control_rows) == 16 and len(plant_rows) == 80,
            "pre_fault_ok": all(row["status"] == "0" for episode in grouped.values() for row in episode[:2]),
            "fault_latched": all(episode[2]["status"] == "4" and episode[2]["latch"] == "1" for episode in grouped.values()),
            "exact_zero_after_fault": all(
                all(float(row[f"command_tau{joint}"]) == 0.0 for joint in range(6))
                for episode in grouped.values() for row in episode[2:]
            ),
            "control_reset_replay": exact_episodes(control_rows, IGNORE),
            "plant_reset_replay": exact_episodes(plant_rows, {"episode"}),
        }
        fault_results.append({"id": fault, "pass": all(checks.values()), "checks": checks})
    replay_checks = []
    for primary in sorted(args.primary.glob("*_control.csv")):
        replay = args.replay / primary.name
        first = rows(primary)
        second = rows(replay)
        replay_checks.append({
            "file": primary.name,
            "pass": len(first) == len(second) and all(
                normalized(a, IGNORE - {"episode"}) == normalized(b, IGNORE - {"episode"})
                for a, b in zip(first, second)
            ),
        })
    for primary in sorted(args.primary.glob("*_plant.csv")):
        replay = args.replay / primary.name
        replay_checks.append({"file": primary.name, "pass": primary.read_bytes() == replay.read_bytes()})
    first_control = output / f"fault_{FAULTS[0]}_control.csv"
    collision = subprocess.run([
        str(args.runner.resolve()), "--model", str(ROOT / config["scene"]),
        "--control-output", str(first_control),
        "--plant-output", str(output / "collision_plant.csv"),
        "--scenario", "hold", "--controller-mode", "phase27_minimal_nmpc",
        "--ticks", "1", "--episodes", "1",
    ], text=True, capture_output=True)
    nonoverwrite = collision.returncode != 0 and "Refusing to overwrite output" in collision.stderr
    summary = {
        "schema_version": 1,
        "pass": all(item["pass"] for item in fault_results + replay_checks) and nonoverwrite,
        "faults": fault_results,
        "fresh_replay": replay_checks,
        "nonoverwrite": {"pass": nonoverwrite, "return_code": collision.returncode, "stderr": collision.stderr.strip()},
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "hardware_data": False, "config": relative(args.config),
        "config_sha256": sha256(args.config), "runner": relative(args.runner),
        "runner_sha256": sha256(args.runner), "wrapper": relative(Path(__file__)),
        "wrapper_sha256": sha256(Path(__file__)), "primary_manifest_sha256": sha256(args.primary / "manifest.json"),
        "replay_manifest_sha256": sha256(args.replay / "manifest.json"),
        "outputs": {path.name: sha256(path) for path in sorted(output.iterdir()) if path.is_file()},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"pass": summary["pass"], "faults": [item["pass"] for item in fault_results], "replay": all(item["pass"] for item in replay_checks), "nonoverwrite": nonoverwrite}))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
