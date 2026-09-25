#!/usr/bin/env bash
# Generate, start an isolated Classic server, capture RGB/depth, then stop that server.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/worlds/generated/validation}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
source /opt/ros/humble/setup.bash
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
export GAZEBO_MASTER_URI="${VALIDATION_MASTER_URI:-http://127.0.0.1:11366}"
export ROS_DOMAIN_ID="${VALIDATION_ROS_DOMAIN_ID:-66}"
export GAZEBO_MODEL_DATABASE_URI=http://127.0.0.1:9
python3 "$ROOT/worlds/generate_metropolis.py" --seed 0 --out "$OUT/city.world"
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$OUT/city.world" > "$OUT/gazebo.log" 2>&1 &
SERVER_PID=$!
cleanup() {
  kill "$SERVER_PID" 2>/dev/null || true
  for ((attempt=0; attempt<10; attempt++)); do
    kill -0 "$SERVER_PID" 2>/dev/null || break
    sleep 1
  done
  kill -KILL "$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
for ((attempt=0; attempt<90; attempt++)); do
  kill -0 "$SERVER_PID" 2>/dev/null || { cat "$OUT/gazebo.log"; exit 1; }
  if rg -q 'Publishing states of gazebo models' "$OUT/gazebo.log"; then break; fi
  sleep 1
done
timeout 540 "$HOME/venv_ros/bin/python" "$ROOT/eval/inspect_metropolis.py" --out "$OUT/review" --manifest "$OUT/city.waypoints.json" "${@:2}"
