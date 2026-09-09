#!/usr/bin/env bash
# docker-entrypoint.sh — 컨테이너 안에서 헤드리스 미션 1회 실행.
# run_fixed.sh와 같은 절차지만, 컨테이너 안에는 venv_ros가 없다(호스트 시스템 파이썬을
# 안 건드려도 되니 전부 시스템 파이썬에 바로 설치했음 — Dockerfile 참고) 그래서 plain python3.
set -e
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"   # Fuel 인터넷 fetch로 멈추는 것 방지
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true

SL="/root/safe_landing"
LOG="$SL/logs/docker_run.log"
FR="$SL/frames/docker_run"
mkdir -p "$SL/logs" "$FR"
: > "$LOG"

echo "[gz] gzserver 헤드리스 시작..." | tee -a "$LOG"
nohup gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    "/root/gz_worlds/residential_delivery.world" >> "$LOG" 2>&1 &
sleep 12

echo "[det] landing_detector 시작..." | tee -a "$LOG"
cd "$SL"
SAVE_FRAMES=1 FRAMES_DIR="$FR" nohup python3 landing_detector.py >> "$LOG" 2>&1 &
sleep 8

echo "[mc] mission_controller 시작..." | tee -a "$LOG"
nohup python3 mission_controller.py >> "$LOG" 2>&1 &
MC_PID=$!

echo "[ok] 전부 기동 — $LOG 를 tail -f 로 보거나, LANDED 될 때까지 기다림" | tee -a "$LOG"

# LANDED 상태가 되거나 최대 5분까지 대기 (재현성 확인용 상한선)
for _ in $(seq 1 150); do
    if grep -q "착륙 완료" "$LOG" 2>/dev/null; then
        echo "[ok] 착륙 완료 확인됨 — 컨테이너 안에서 미션이 끝까지 돌아감을 검증함" | tee -a "$LOG"
        break
    fi
    sleep 2
done

echo "마지막 로그 20줄:"
tail -20 "$LOG"
echo ""
echo "프레임: $FR ($(ls "$FR" 2>/dev/null | wc -l)장)"
