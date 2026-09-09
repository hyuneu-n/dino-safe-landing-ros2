#!/usr/bin/env bash
# run_fixed.sh — 고정 로그명(demo_latest.log)으로 헤드리스 1회 미션
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
SL="$HOME/safe_landing"
LOG="$SL/logs/demo_latest.log"
FR="$SL/frames/demo_latest"
rm -rf "$FR"; mkdir -p "$FR" "$SL/logs"
: > "$LOG"

# hard clean
pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null; pkill -9 -x gazebo 2>/dev/null
pkill -9 -f landing_detector.py 2>/dev/null
pkill -9 -f mission_controller.py 2>/dev/null
pkill -9 -f frame_saver.py 2>/dev/null
fuser -k -9 11345/tcp 2>/dev/null
sleep 3

echo "[gz] starting gzserver" >> "$LOG"
nohup gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  "$HOME/gz_worlds/residential_delivery.world" >> "$LOG" 2>&1 &
sleep 11

echo "[det] starting landing_detector" >> "$LOG"
nohup bash -c "source '$HOME/venv_ros/bin/activate'; cd '$SL'; SAVE_FRAMES=1 FRAMES_DIR='$FR' python3 landing_detector.py" >> "$LOG" 2>&1 &
sleep 6

echo "[mc] starting mission_controller" >> "$LOG"
nohup bash -c "cd '$SL'; python3 mission_controller.py" >> "$LOG" 2>&1 &
sleep 1

echo "[fs] starting frame_saver" >> "$LOG"
nohup bash -c "cd '$SL'; FRAMES_DIR='$FR' python3 frame_saver.py" >> "$LOG" 2>&1 &

echo "[ok] all dispatched -> $LOG" >> "$LOG"
