#!/usr/bin/env bash
# Gazebo Fuel(OpenRobotics) 모델 다운로드 → ~/.gazebo/models/  (model://Name 으로 참조 가능)
set -u
DEST="$HOME/.gazebo/models"
mkdir -p "$DEST"
BASE="https://fuel.gazebosim.org/1.0/OpenRobotics/models"
# 이름(공백은 %20)
MODELS=(
  "House%201" "House%202" "House%203"
  "Pine%20Tree" "Oak%20tree"
  "Lamp%20Post" "Hatchback" "Pickup" "Mailbox"
  "Construction%20Cone" "Fire%20hydrant"
)
ok=0; fail=0
for m in "${MODELS[@]}"; do
  name="${m//%20/ }"
  if [ -f "$DEST/$name/model.sdf" ]; then echo "[skip] $name (이미 있음)"; ok=$((ok+1)); continue; fi
  tmp=$(mktemp /tmp/fuel_XXXX.zip)
  if timeout 120 wget -q -O "$tmp" "$BASE/$m.zip" && [ -s "$tmp" ]; then
    mkdir -p "$DEST/$name"
    if unzip -oq "$tmp" -d "$DEST/$name"; then echo "[ok]   $name"; ok=$((ok+1)); else echo "[fail] $name (압축해제)"; fail=$((fail+1)); fi
  else
    echo "[fail] $name (다운로드)"; fail=$((fail+1))
  fi
  rm -f "$tmp"
done
echo "=== 완료: 성공 $ok / 실패 $fail ==="
ls "$DEST"
