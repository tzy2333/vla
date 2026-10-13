#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if ! command -v tmux >/dev/null 2>&1; then
  exec .venv/bin/python -u monitor.py "$@"
fi
if ! tmux has-session -t collector-display 2>/dev/null; then
  tmux new-session -d -s collector-display -x 140 -y 42 "$(pwd)/.venv/bin/python -u $(pwd)/monitor.py --rate 40 --poll 15"
fi
exec tmux attach-session -t collector-display
