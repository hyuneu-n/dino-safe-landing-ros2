#!/usr/bin/env bash
# run_demo.sh — 졸작 v2 데모 한 방 실행 (WSL2 Ubuntu-22.04)
#   가제보(월드+드론) + 착륙지탐지 노드 + 미션컨트롤러 + RViz2
# 사용: bash ~/safe_landing/run_demo.sh        (RViz 없이: bash ~/safe_landing/run_demo.sh norviz)
set -e
# GPU 모드 — Fuel 메시 (House/Car/Tree 등) 빠르게 렌더
unset LIBGL_ALWAYS_SOFTWARE
unset GALLIUM_DRIVER
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"   # 죽은 온라인 모델DB 즉시 거부 (행 방지)
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
SL="$HOME/safe_landing"
mkdir -p "$SL/logs" "$SL/frames"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$SL/logs/demo_$TS.log"
export FRAMES_DIR="$SL/frames/$TS"
mkdir -p "$FRAMES_DIR"
echo "=== 로그: $LOG ===" | tee "$LOG"
echo "=== 프레임: $FRAMES_DIR ===" | tee -a "$LOG"

echo "[1/5] 기존 Gazebo/RViz 종료..."
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f gazebo   2>/dev/null || true
pkill -f rviz2    2>/dev/null || true
pkill -f landing_detector 2>/dev/null || true
pkill -f mission_controller 2>/dev/null || true
sleep 1

echo "[2/5] Gazebo (월드+드론, ROS2 플러그인) 실행..."
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  "$HOME/gz_worlds/residential_delivery.world" >> "$LOG" 2>&1 &
GZ=$!
sleep 9

echo "[3/5] 착륙지 탐지 노드 (DINOv2) 실행..."
( source "$HOME/venv_ros/bin/activate"; cd "$SL"; SAVE_FRAMES=1 python3 landing_detector.py 2>&1 | tee -a "$LOG" ) &
DET=$!
sleep 4

echo "[4/6] 미션 컨트롤러 (드론 비행 + TF) 실행..."
( cd "$SL"; python3 mission_controller.py 2>&1 | tee -a "$LOG" ) &
MC=$!
sleep 1

echo "[5/6] Frame Saver (모든 카메라/오버레이 → 개별 PNG) 실행..."
( cd "$SL"; FRAMES_DIR="$FRAMES_DIR" python3 frame_saver.py 2>&1 | tee -a "$LOG" ) &
FS=$!
sleep 1

RV=""
if [ "$1" != "norviz" ]; then
  echo "[6/6] RViz2 실행..."
  chmod 444 "$SL/demo.rviz"     # RViz가 닫힐 때 config 덮어쓰지 못하게 (FrontDepth/IsoCam 패널 보존)
  rviz2 -d "$SL/demo.rviz" &
  RV=$!
fi

echo
echo "=== 실행 중 ==="
echo "  Gazebo  : 드론이 고층빌딩 사이 통과 → 목적지 상공 → 스캔 → 안전지점 하강 → 착륙"
echo "  RViz2   : 드론 카메라 / overlay(녹=안전, 적=장애물) / 착륙후보 디스크 / 선택점(녹색 화살표) / depth 클라우드"
echo "  로그    : $LOG  (콘솔 + 노드 출력 다 저장됨)"
echo "  프레임  : $FRAMES_DIR/  (모든 카메라/패널 번호매김 PNG)"
echo "  영상 만들기 (Ctrl+C 후, 패널별 MP4):"
echo "    cd $FRAMES_DIR"
echo "    for p in overlay pca downcam chasecam isocam frontview frontdepth; do"
echo "      ffmpeg -y -framerate \$( [ \$p = overlay ] || [ \$p = pca ] && echo 2 || echo 5 ) \\\\"
echo "        -i \${p}_%04d.png -c:v libx264 -pix_fmt yuv420p \${p}.mp4 2>/dev/null"
echo "    done"
echo "  상태    : ros2 topic echo /mission/status"
echo "  종료    : Ctrl+C"
trap "kill $GZ $DET $MC $FS $RV 2>/dev/null" EXIT
wait
