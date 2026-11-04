#!/usr/bin/env bash
# Standalone review-index generator: readable-name symlinks for RoboAPI task recordings.
# Usage: ./scripts/make_review.sh [--dry-run|--clean]
exec /home/ps/roboapi/.venv/bin/python /home/ps/roboapi/scripts/link_tasks.py "$@"
