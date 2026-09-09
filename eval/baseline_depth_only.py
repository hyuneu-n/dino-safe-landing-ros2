#!/usr/bin/env python3
"""
baseline_depth_only.py — PX4 safe_landing_planner 방식 재구현 (depth-only, semantic 없음)

원본 PX4-Avoidance/safe_landing_planner는 2024-04 archived되어 그대로 실행할 수 없으므로,
공개 문서(https://docs.px4.io/main/en/computer_vision/safe_landing.html, BSD-3)에 나온
알고리즘을 depth 이미지에 맞게 재구현했다:
  드론 footprint 크기의 윈도우로 depth 맵을 훑어, 윈도우 내 표준편차(=울퉁불퉁한 정도)가
  임계값 이하인 곳만 "평평함"으로 표시하고, 그 중 목적지에 가장 가까운 곳을 고른다.
RGB나 의미(semantic) 정보는 전혀 쓰지 않는다 — 그래서 색이 도로와 다른 잔디밭이나 낮은
장애물의 "표면 재질"은 구분 못 하고 오직 "평평한가"만 본다. 이게 우리 방법(DINOv2 색상
군집 + depth 융합)과 비교했을 때 이 baseline이 실패하는 전형적인 이유가 된다
(예: 잔디밭은 평평해서 "안전"으로 잘못 고를 수 있음).
"""
import numpy as np


def pick_landing_point(depth, target_xy, h_alt, odom_xy, footprint_px=24, std_thresh=0.12,
                        min_valid_frac=0.6):
    """depth: HxW float32 (미터, NaN/inf = 무효). footprint_px: 드론 반경에 해당하는 half-window(px).
    반환: (dist_to_target_m, u, v) 또는 후보가 없으면 None."""
    from camera import pixel_to_world
    H, W = depth.shape
    win = footprint_px
    step = max(2, win // 2)  # 전 픽셀 스캔 대신 그리드 샘플링 (PX4도 그리드 단위로 스캔함)
    best = None
    for v in range(win, H - win, step):
        for u in range(win, W - win, step):
            patch = depth[v - win:v + win, u - win:u + win]
            valid = patch[np.isfinite(patch)]
            if valid.size < (2 * win) ** 2 * min_valid_frac:
                continue
            if float(np.std(valid)) > std_thresh:
                continue
            wx, wy = pixel_to_world(u, v, h_alt, odom_xy)
            d = float(np.hypot(wx - target_xy[0], wy - target_xy[1]))
            if best is None or d < best[0]:
                best = (d, u, v)
    return best
