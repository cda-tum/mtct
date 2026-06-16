#!/usr/bin/env python3
"""Add station_delay_weight=1 to test instance problem_data.json files when missing."""

from __future__ import annotations

import json
from pathlib import Path

TARGET_KEY_ORDER = (
    "lambda",
    "station_delay_weight",
    "train_weights",
    "train_optional",
)


def reorder_keys(data: dict) -> dict:
    """Return a copy of data with canonical key order first."""
    ordered: dict = {}

    for key in TARGET_KEY_ORDER:
        if key in data:
            ordered[key] = data[key]

    for key, value in data.items():
        if key not in ordered:
            ordered[key] = value

    return ordered


def update_problem_data_file(file_path: Path) -> bool:
    """Return True if the file was modified."""
    with file_path.open("r", encoding="utf-8") as f:
        original_data = json.load(f)

    data = dict(original_data)
    if "station_delay_weight" not in data:
        data["station_delay_weight"] = 1

    ordered_data = reorder_keys(data)

    modified = (
        original_data != ordered_data
        or list(original_data.keys()) != list(ordered_data.keys())
    )
    if not modified:
        return False

    with file_path.open("w", encoding="utf-8", newline="\n") as f:
        json.dump(ordered_data, f, indent=2)
        f.write("\n")

    return True


def main() -> int:
    repo_root = Path(__file__).resolve().parent
    instances_dir = repo_root / "test" / "data" / "instances"

    if not instances_dir.exists():
        raise FileNotFoundError(f"Directory not found: {instances_dir}")

    problem_files = sorted(instances_dir.rglob("problem_data.json"))

    updated_files: list[Path] = []
    for file_path in problem_files:
        if update_problem_data_file(file_path):
            updated_files.append(file_path)

    print(f"Scanned {len(problem_files)} files.")
    print(f"Updated {len(updated_files)} files.")
    for updated in updated_files:
        print(updated.relative_to(repo_root).as_posix())

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
