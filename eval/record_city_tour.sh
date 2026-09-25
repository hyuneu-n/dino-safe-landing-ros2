#!/usr/bin/env bash
# Isolated renderer; owns only the server it starts. Optional duration/fps follow OUT.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/artifacts/metropolis_tour}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
source /opt/ros/humble/setup.bash
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
export GAZEBO_MASTER_URI="${TOUR_MASTER_URI:-http://127.0.0.1:11469}"
export ROS_DOMAIN_ID="${TOUR_ROS_DOMAIN_ID:-169}"
export GAZEBO_MODEL_DATABASE_URI=http://127.0.0.1:9
python3 "$ROOT/worlds/generate_metropolis.py" --seed 0 --out "$OUT/city.world"
python3 "$ROOT/eval/record_city_tour.py" --prepare "$OUT/city.world" --out "$OUT/tour.world"
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$OUT/tour.world" > "$OUT/gazebo.log" 2>&1 &
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
timeout 1200 "$HOME/venv_ros/bin/python" "$ROOT/eval/record_city_tour.py" --out "$OUT/metropolis_flythrough.mp4" "${@:2}"
