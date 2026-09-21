#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export MUJOCO_GL=egl
OPEN=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --open) OPEN=1; shift ;;
    --api-key)
      [ "$#" -ge 2 ] || { echo "--api-key requires a value" >&2; exit 2; }
      export TYPESAFE_API_KEY="$2"; shift 2 ;;
    --key-file)
      [ "$#" -ge 2 ] || { echo "--key-file requires a path" >&2; exit 2; }
      export JEV_KEY_FILE="$2"; shift 2 ;;
    *) echo "Usage: bash start.sh [--open] [--api-key KEY] [--key-file PATH]" >&2; exit 2 ;;
  esac
done
if curl -fsS http://127.0.0.1:8094/api/state >/dev/null 2>&1; then
  echo "Jev MuJoCo Lab already running: http://127.0.0.1:8094"
else
  .venv/bin/python build_scene.py
  nohup .venv/bin/uvicorn app:app --host 127.0.0.1 --port 8094 > logs/web.log 2>&1 &
  echo $! > logs/web.pid
  echo "Started paused: http://127.0.0.1:8094"
fi
if [ "$OPEN" = "1" ]; then
  for attempt in {1..30}; do
    if curl -fsS http://127.0.0.1:8094/api/state >/dev/null 2>&1; then break; fi
    sleep 0.3
  done
  DISPLAY=${DISPLAY:-:1} XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} google-chrome --new-window http://127.0.0.1:8094 > logs/browser.log 2>&1 &
fi
