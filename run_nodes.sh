#!/usr/bin/env bash
# run_nodes.sh — Gazebo 이미 떠 있다고 가정. 노드만 띄움 (detector / mission / frame_saver / RViz)
# 사용: bash ~/safe_landing/run_nodes.sh        (RViz 없이: bash ~/safe_landing/run_nodes.sh norviz)
# (set -e 일부러 안 씀)
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
source /opt/ros/humble/setup.bash
SL="$HOME/safe_landing"
mkdir -p "$SL/logs" "$SL/frames"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$SL/logs/nodes_$TS.log"
export FRAMES_DIR="$SL/frames/$TS"
mkdir -p "$FRAMES_DIR"
echo "=== 로그: $LOG ===" | tee "$LOG"
echo "=== 프레임: $FRAMES_DIR ===" | tee -a "$LOG"

# Gazebo 떠있는지 확인
if ! pgrep -ax gzserver >/dev/null && ! pgrep -ax gazebo >/dev/null; then
  echo "❌ Gazebo가 안 돌고 있음. 먼저 'bash ~/safe_landing/start_gazebo.sh' 실행"
  exit 1
fi
echo "✅ Gazebo 감지됨"

echo "[1/4] 기존 노드 정리..."
pkill -9 -x rviz2 2>/dev/null || true
pkill -9 -f landing_detector.py 2>/dev/null || true
pkill -9 -f mission_controller.py 2>/dev/null || true
pkill -9 -f frame_saver.py 2>/dev/null || true
sleep 1

echo "[2/4] 착륙지 탐지 (DINOv2)..."
( source "$HOME/venv_ros/bin/activate"; cd "$SL"; SAVE_FRAMES=1 python3 landing_detector.py 2>&1 | tee -a "$LOG" ) &
DET=$!
sleep 4

echo "[3/4] 미션 컨트롤러 (드론 비행)..."
( cd "$SL"; python3 mission_controller.py 2>&1 | tee -a "$LOG" ) &
MC=$!
sleep 1

echo "[4/4] Frame Saver (모든 패널 → PNG)..."
( cd "$SL"; FRAMES_DIR="$FRAMES_DIR" python3 frame_saver.py 2>&1 | tee -a "$LOG" ) &
FS=$!
sleep 1

RV=""
if [ "$1" != "norviz" ]; then
  echo "[+] RViz2..."
  chmod 444 "$SL/demo.rviz"
  rviz2 -d "$SL/demo.rviz" 2>/dev/null &
  RV=$!
fi

echo
echo "=== 실행 중 ==="
echo "  드론 비행 시작 → 빌딩 회피 → 목적지 스캔 → 보행자 진입 → 재평가 → 착륙 (~1~2분)"
echo "  로그   : $LOG"
echo "  프레임 : $FRAMES_DIR/  (overlay/pca/downcam/chasecam/isocam/frontview/frontdepth)"
echo "  상태   : ros2 topic echo /mission/status"
echo "  종료   : Ctrl+C  (Gazebo는 따로 — 그 창 X 또는 'pkill -9 -x gzserver gzclient gazebo')"
echo
echo "  영상 만들기 (Ctrl+C 후, 패널별 부드러운 MP4):"
echo "    cd $FRAMES_DIR"
echo "    for p in downcam chasecam isocam frontview frontdepth; do"
echo "      ffmpeg -y -framerate 15 -i \${p}_%04d.png -c:v libx264 -pix_fmt yuv420p \${p}.mp4 2>/dev/null"
echo "    done"
echo "    for p in overlay pca; do"
echo "      ffmpeg -y -framerate 2 -i \${p}_%04d.png -c:v libx264 -pix_fmt yuv420p \${p}.mp4 2>/dev/null"
echo "    done && ls *.mp4"
echo
echo "  4분할 그리드 영상(chasecam + downcam + overlay + frontdepth 동시):"
echo "    ffmpeg -y \\\\"
echo "      -framerate 15 -i chasecam_%04d.png  -framerate 15 -i downcam_%04d.png \\\\"
echo "      -framerate 15 -i frontdepth_%04d.png -framerate 2 -i overlay_%04d.png \\\\"
echo "      -filter_complex '[0:v]scale=640:400[a];[1:v]scale=640:480[b];[2:v]scale=640:480[c];[3:v]scale=640:480[d];[a][b]hstack[top];[c][d]hstack[bot];[top][bot]vstack[v]' \\\\"
echo "      -map '[v]' -c:v libx264 -pix_fmt yuv420p grid.mp4"
trap "kill $DET $MC $FS $RV 2>/dev/null" EXIT
wait
