#!/usr/bin/env bash
# run_headless.sh — GUI 없이 gzserver + 노드 + 프레임저장 (1회 미션)
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
SL="$HOME/safe_landing"
mkdir -p "$SL/logs" "$SL/frames"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$SL/logs/headless_$TS.log"
export FRAMES_DIR="$SL/frames/$TS"
mkdir -p "$FRAMES_DIR"
echo "LOG=$LOG"  | tee "$SL/logs/last_headless.txt"
echo "FRAMES=$FRAMES_DIR" | tee -a "$SL/logs/last_headless.txt"
echo "TS=$TS" | tee -a "$SL/logs/last_headless.txt"

pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null; pkill -9 -x gazebo 2>/dev/null
pkill -9 -f landing_detector.py 2>/dev/null
pkill -9 -f mission_controller.py 2>/dev/null
pkill -9 -f frame_saver.py 2>/dev/null
fuser -k -9 11345/tcp 2>/dev/null
sleep 3

echo "[gz] gzserver 헤드리스 시작..." >> "$LOG"
nohup gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  "$HOME/gz_worlds/residential_delivery.world" >> "$LOG" 2>&1 &
sleep 11

echo "[det] landing_detector 시작..." >> "$LOG"
nohup bash -c "source '$HOME/venv_ros/bin/activate'; cd '$SL'; SAVE_FRAMES=1 FRAMES_DIR='$FRAMES_DIR' python3 landing_detector.py" >> "$LOG" 2>&1 &
sleep 5

echo "[mc] mission_controller 시작..." >> "$LOG"
nohup bash -c "cd '$SL'; python3 mission_controller.py" >> "$LOG" 2>&1 &
sleep 1

echo "[fs] frame_saver 시작..." >> "$LOG"
nohup bash -c "cd '$SL'; FRAMES_DIR='$FRAMES_DIR' python3 frame_saver.py" >> "$LOG" 2>&1 &

echo "ALL_STARTED TS=$TS" >> "$LOG"
