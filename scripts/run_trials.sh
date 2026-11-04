#!/usr/bin/env bash
# Always run the trial runner with the project venv (it owns PyYAML, i2rt and roboapi).
exec /home/ps/roboapi/.venv/bin/python /home/ps/roboapi/scripts/trial_runner.py "$@"
