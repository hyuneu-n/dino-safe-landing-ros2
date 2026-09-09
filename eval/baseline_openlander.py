#!/usr/bin/env python3
"""
baseline_openlander.py — OpenLander (RGB-only DNN 세그멘테이션) 착륙지 베이스라인.

원본: https://github.com/stephansturges/OpenLander (MIT License, Stephan Sturges)
가중치: eval/models/openlander_end2end.onnx — 출처는 models/download_openlander.sh 참고.
  ⚠️ 레포 자체의 models/*.blob 은 Luxonis OAK 카메라의 Myriad X VPU 전용 컴파일 바이너리라
  onnxruntime/일반 PC에서 못 돌린다 (README의 "ONNX 모델" 표현과 달리, 진짜 이식 가능한
  .onnx 파일은 저자가 올린 HuggingFace Streamlit 데모(app.py)의 리소스로만 공개돼 있음).
  전처리/후처리는 그 app.py의 mosaic_crop()/process_image() 로직을 그대로 이식했다.

출력 클래스 (모델이 이미 픽셀별로 argmax된 정수 맵을 냄): 0=obstacle, 1=safe, 2=human.
depth는 전혀 쓰지 않는다 — "평평한지"는 모르고 오직 "이 표면이 안전해 보이는지"만 본다.
그래서 이 baseline이 실패하는 전형적인 이유는 우리 baseline_depth_only.py와 정반대다:
낮은 차고 지붕(low_garage)처럼 색만 보면 도로와 비슷한 표면은 솟아 있어도 "안전"으로
잘못 고를 수 있다.
"""
import os
import numpy as np
import cv2
import onnxruntime as ort

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models",
                           "openlander_end2end.onnx")
TILE = 416
OBSTACLE_CLASS, SAFE_CLASS, HUMAN_CLASS = 0, 1, 2


class OpenLanderBaseline:
    def __init__(self, model_path=MODEL_PATH):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"{model_path} 없음 — eval/models/download_openlander.sh 를 먼저 실행하세요.")
        self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

    def _infer_tile(self, tile_bgr):
        blob = cv2.dnn.blobFromImage(tile_bgr, 1.0 / 255.0, (TILE, TILE), swapRB=True, crop=False)
        out = self.session.run(None, {self.input_name: blob.astype(np.float32)})[0]
        return out[0, 0].astype(np.uint8)  # (TILE,TILE) 클래스 인덱스 맵 {0,1,2}

    def segment(self, rgb):
        """rgb: HxWx3 uint8 RGB(예: down_cam 프레임). 반환: 원본 크기의 클래스 인덱스 맵.
        원본 app.py의 mosaic 모드처럼 416px 타일로 나눠 처리 후 이어붙인다 (임의 해상도 지원)."""
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        h, w = bgr.shape[:2]
        ph = (TILE - h % TILE) % TILE
        pw = (TILE - w % TILE) % TILE
        padded = cv2.copyMakeBorder(bgr, 0, ph, 0, pw, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        rows, cols = padded.shape[0] // TILE, padded.shape[1] // TILE
        out = np.zeros((padded.shape[0], padded.shape[1]), np.uint8)
        for i in range(rows):
            for j in range(cols):
                tile = padded[i * TILE:(i + 1) * TILE, j * TILE:(j + 1) * TILE]
                out[i * TILE:(i + 1) * TILE, j * TILE:(j + 1) * TILE] = self._infer_tile(tile)
        return out[:h, :w]

    def pick_landing_point(self, rgb, target_xy, h_alt, odom_xy, min_area_px=30):
        """safe(class 1) 연결요소 중 target에 가장 가까운 것의 중심 픽셀을 고른다.
        반환: (dist_to_target_m, u, v) 또는 안전 영역이 없으면 None."""
        from camera import pixel_to_world
        cls = self.segment(rgb)
        safe = (cls == SAFE_CLASS).astype(np.uint8)
        n, _labels, stats, centroids = cv2.connectedComponentsWithStats(safe, connectivity=4)
        best = None
        for i in range(1, n):  # 0 = 배경
            if stats[i, cv2.CC_STAT_AREA] < min_area_px:
                continue
            cu, cv_ = centroids[i]
            wx, wy = pixel_to_world(cu, cv_, h_alt, odom_xy)
            d = float(np.hypot(wx - target_xy[0], wy - target_xy[1]))
            if best is None or d < best[0]:
                best = (d, float(cu), float(cv_))
        return best
