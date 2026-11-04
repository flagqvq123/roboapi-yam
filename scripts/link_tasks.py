#!/usr/bin/env python3
"""Create readable-name symlinks for RoboAPI task recordings (read-only; never touches logs/tasks).

Each link lives in ~/roboapi/review and points at the real video.mp4. Names combine the task,
trial number and verdict from our trial CSVs when they exist, otherwise the timestamp and a short
task id. Deleting the review directory removes every link and nothing else.

Usage:
  python scripts/link_tasks.py            # add/refresh links
  python scripts/link_tasks.py --dry-run  # show what would be created
  python scripts/link_tasks.py --clean    # remove links created by this script, then relink
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import pathlib
import re
import sys

HOME = pathlib.Path.home() / "roboapi"
TASKS = HOME / "logs" / "tasks"
CSVS = HOME / "trials"
REVIEW = HOME / "review"
LINK_PREFIX = ""


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def load_trial_index():
    """task_id -> (task, trial, result, rating) from the trial CSVs."""
    index = {}
    if not CSVS.is_dir():
        return index
    for path in sorted(CSVS.glob("*.csv")):
        try:
            with path.open(newline="", encoding="utf-8") as fh:
                for row in csv.DictReader(fh):
                    task_id = (row.get("task_id") or "").strip()
                    if not task_id:
                        continue
                    index[task_id] = (row.get("task") or "task", row.get("trial") or "?",
                                      row.get("result") or "?", row.get("rating") or "")
        except Exception:
            continue
    return index


def slug(text, limit=24):
    text = re.sub(r"[^0-9A-Za-z\u4e00-\u9fff]+", "-", str(text or "")).strip("-")
    return text[:limit] or "task"


def target_name(meta, trials):
    task_id = meta.get("id") or ""
    created = meta.get("created_at") or 0
    stamp = datetime.datetime.fromtimestamp(float(created)).strftime("%Y-%m-%d_%H%M")
    trial = trials.get(task_id)
    if trial:
        task, number, result, rating = trial
        name = f"{stamp}_{slug(task)}_t{str(number).zfill(2)}_{result}"
        if rating:
            name += f"_r{rating}"
        return name + ".mp4"
    return f"{stamp}_{task_id[:8]}.mp4"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--clean", action="store_true", help="remove previously created links first")
    parser.add_argument("--review-dir", default=str(REVIEW))
    args = parser.parse_args()
    review = pathlib.Path(args.review_dir).expanduser()
    review.mkdir(parents=True, exist_ok=True)
    trials = load_trial_index()

    if args.clean:
        removed = 0
        for link in review.glob("*.mp4"):
            if link.is_symlink():
                if args.dry_run:
                    print("would remove", link.name)
                else:
                    link.unlink()
                removed += 1
        if not args.dry_run:
            print("removed", removed, "links")

    created = skipped = missing = 0
    for task_dir in sorted(TASKS.glob("*/")):
        meta = read_json(task_dir / "metadata.json")
        video = task_dir / "video.mp4"
        if meta is None:
            missing += 1
            continue
        if not video.exists():
            missing += 1
            continue
        name = target_name(meta, trials)
        link = review / name
        relative = pathlib.Path("../logs/tasks") / task_dir.name / "video.mp4"
        if link.is_symlink() and link.readlink() == relative:
            skipped += 1
            continue
        if link.exists() or link.is_symlink():
            if args.dry_run:
                print("would replace", name)
            else:
                link.unlink()
        if args.dry_run:
            print("would link", name, "->", relative)
        else:
            link.symlink_to(relative)
        created += 1

    print()
    print("review dir:", review)
    print("created:", created, "skipped:", skipped, "no-video:", missing)
    if not args.dry_run and created:
        sample = sorted(review.glob("*.mp4"))[:5]
        for link in sample:
            print("  ", link.name)


if __name__ == "__main__":
    sys.exit(main())
