#!/usr/bin/env bash
# download_openlander.sh — OpenLander baseline 가중치 받기 (재현용, 저장소엔 안 커밋함)
#
# 출처: https://huggingface.co/spaces/StephanST/OpenLanderONNXonline (MIT)
#   -> app.py 가 세 모델 중 "Embedded model better trained: DeeplabV3+, MobilenetV2, 416px"로
#      쓰는 파일이 이것. GitHub 레포(stephansturges/OpenLander)의 models/*.blob 은 Luxonis
#      OAK 카메라 VPU 전용 컴파일 바이너리라 onnxruntime으로 못 돌려서, 대신 저자가 같이
#      올린 이 Streamlit 데모의 순수 .onnx 파일을 쓴다.
set -e
cd "$(dirname "$0")"
URL="https://huggingface.co/spaces/StephanST/OpenLanderONNXonline/resolve/main/20230608_onnx_416_mbnv2_dl3/end2end.onnx"
OUT="openlander_end2end.onnx"
echo "[다운로드] $URL -> $OUT"
curl -sL "$URL" -o "$OUT"
python3 -c "
import onnxruntime as ort
s = ort.InferenceSession('$OUT', providers=['CPUExecutionProvider'])
print('OK — input', s.get_inputs()[0].shape, 'output', s.get_outputs()[0].shape)
"
