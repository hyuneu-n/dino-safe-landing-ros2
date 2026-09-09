#!/usr/bin/env bash
# start_gazebo.sh — Gazebo만 따로 띄움 (녹화 시작 후 → run_nodes.sh)
# (set -e 일부러 안 씀 — pkill이 죽일 프로세스 없으면 1 반환해서 스크립트 죽음)
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"

echo "[clean] 기존 Gazebo/PX4/port 정리..."
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
pkill -9 -x gazebo   2>/dev/null || true
pkill -9 -x px4      2>/dev/null || true
pkill -9 -f sitl_run 2>/dev/null || true
# port 11345를 잡고 있는 프로세스 강제 종료
fuser -k -9 11345/tcp 2>/dev/null || true
sleep 3
# 포트 진짜 비었는지 확인 (최대 8초)
for i in 1 2 3 4 5 6 7 8; do
  if ! ss -ltn 2>/dev/null | grep -q ':11345'; then break; fi
  echo "  port 11345 still busy ($i/8), waiting..."
  sleep 1
done
if ss -ltn 2>/dev/null | grep -q ':11345'; then
  echo "❌ port 11345 안 풀림. WSL 재시작 필요할 수도: 'wsl --shutdown' (PowerShell)"
  exit 1
fi
echo "✅ 정리 완료 — Gazebo 띄움"

echo
echo "▶ Gazebo 창 뜨면:"
echo "   1) OBS / 녹화도구로 Gazebo 창 캡처 시작"
echo "   2) 새 WSL 터미널 열고:  bash ~/safe_landing/run_nodes.sh"
echo "      → 드론 비행 + 탐지 + RViz 시작됨"
echo

gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  "$HOME/gz_worlds/residential_delivery.world"
