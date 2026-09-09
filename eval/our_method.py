#!/usr/bin/env python3
"""
our_method.py — landing_detector.py의 DINOv2+depth 착륙지 선정 알고리즘을 ROS 노드 밖에서도
호출할 수 있게 뽑아낸 순수 함수. eval/run_comparison.py 전용.

landing_detector.LandingDetector.process()의 1~4단계(DINO 특징+군집 → 장애물 마스크 →
주 착륙표면 판정 → 침식+연결요소 → 후보 스코어링)를 numpy 레벨에서 그대로 복제했다.
발행(publish)/오버레이 그리기 같은 ROS·시각화 전용 코드만 뺐다.

⚠️ landing_detector.py의 알고리즘 상수/로직이 바뀌면 이 파일도 손으로 맞춰야 한다 — 배포
핵심 파일(landing_detector.py)에 eval 전용 리팩터링을 강제하는 리스크를 피하려고 일부러
분리해뒀다 (CONTEXT.md에 남겨둔 알려진 트레이드오프).
"""
import math
from collections import deque
import numpy as np
from PIL import Image as PImage

IMG_W, IMG_H = 640, 480
FX = (IMG_W / 2.0) / math.tan(1.396 / 2.0)
FY = FX
CX, CY = IMG_W / 2.0, IMG_H / 2.0
DRONE_FOOTPRINT_M = 1.0
OBSTACLE_DEPTH_MARGIN = 0.45
GROUND_COLOR_THRESH = 70.0
W_NEAR, W_FLAT, W_CLEAR, W_AREA = 0.40, 0.20, 0.25, 0.15


def pixel_to_world(u, v, h, odom_xy):
    off_a = (CX - u) / FX * h
    off_b = (CY - v) / FY * h
    ox, oy = odom_xy
    return ox + off_b, oy + off_a


def _block_median(a, hp, wp):
    H, W = a.shape
    ys = np.linspace(0, H, hp + 1).astype(int)
    xs = np.linspace(0, W, wp + 1).astype(int)
    out = np.full((hp, wp), np.nan)
    for i in range(hp):
        for j in range(wp):
            blk = a[ys[i]:ys[i + 1], xs[j]:xs[j + 1]]
            if np.isfinite(blk).any():
                out[i, j] = np.nanmedian(blk)
    return out


def _erode(mask, r):
    if r <= 0:
        return mask.copy()
    from numpy.lib.stride_tricks import sliding_window_view as swv
    m = mask.astype(np.uint8)
    m = swv(np.pad(m, ((0, 0), (r, r))), 2 * r + 1, axis=1).min(-1)
    m = swv(np.pad(m, ((r, r), (0, 0))), 2 * r + 1, axis=0).min(-1)
    return m.astype(bool)


def _connected_components(mask):
    H, W = mask.shape
    lab = np.zeros((H, W), np.int32)
    out, cur = [], 0
    ys_, xs_ = np.where(mask)
    for y, x in zip(ys_, xs_):
        if lab[y, x]:
            continue
        cur += 1
        q = deque([(y, x)])
        lab[y, x] = cur
        sx = sy = n = 0
        while q:
            cy, cx = q.popleft()
            sx += cx
            sy += cy
            n += 1
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    q.append((ny, nx))
        out.append((sx / n, sy / n, n))
    return out


def pick_landing_point(seg, rgb, depth, odom_xy, h_alt, target_xy):
    """seg: dino_seg.DinoSeg 인스턴스 (호출자가 미리 한 번만 로드해서 재사용할 것 — 매 호출마다
    새로 만들면 극도로 느림). 반환: (score, u, v, wx, wy) 또는 후보가 없으면 None."""
    feats, (hp, wp) = seg.extract(rgb, side=448)
    labels_lo, _ = seg.kmeans(feats, k=5)

    rgb_lo = np.asarray(PImage.fromarray(rgb).resize((wp, hp), PImage.BILINEAR)).astype(float)
    dvalid = np.where(np.isfinite(depth), depth, np.nan)
    d_lo = _block_median(dvalid, hp, wp)
    ground_dist = np.nanmedian(d_lo)
    if not np.isfinite(ground_dist):
        ground_dist = h_alt

    obstacle_lo = (ground_dist - np.nan_to_num(d_lo, nan=ground_dist)) > OBSTACLE_DEPTH_MARGIN

    flat_ground = {}
    for c in range(5):
        m = labels_lo == c
        if m.sum() < 4:
            continue
        obst_frac = obstacle_lo[m].mean()
        d_dev = np.nanstd(d_lo[m]) if np.isfinite(np.nanstd(d_lo[m])) else 9.9
        if obst_frac < 0.25 and d_dev < 0.6:
            flat_ground[c] = (m.sum(), rgb_lo[m].mean(0))
    safe_lo = np.zeros((hp, wp), bool)
    if flat_ground:
        primary = max(flat_ground, key=lambda c: flat_ground[c][0])
        pcol = flat_ground[primary][1]
        for c, (sz, col) in flat_ground.items():
            if np.linalg.norm(col - pcol) <= GROUND_COLOR_THRESH:
                safe_lo[labels_lo == c] = True
    safe_lo &= ~obstacle_lo

    def up(a):
        return np.asarray(PImage.fromarray(a.astype(np.uint8)).resize((IMG_W, IMG_H), PImage.NEAREST)) > 0
    safe = up(safe_lo)

    m_per_px = (2.0 * h_alt * math.tan(1.396 / 2.0)) / IMG_W
    rad_px = max(2, min(int(DRONE_FOOTPRINT_M / max(m_per_px, 1e-3)), IMG_W // 8))
    comps = _connected_components(_erode(safe, rad_px))

    cands = []
    for (cu, cv, area_px) in comps:
        if area_px < math.pi * rad_px * rad_px:
            continue
        wx, wy = pixel_to_world(cu, cv, h_alt, odom_xy)
        near = math.exp(-np.linalg.norm(np.array([wx, wy]) - np.array(target_xy)) / 6.0)
        lu, lv = int(cu * wp / IMG_W), int(cv * hp / IMG_H)
        ld = d_lo[max(0, lv - 2):lv + 3, max(0, lu - 2):lu + 3]
        sd = np.nanstd(ld)
        flat = 1.0 / (1.0 + (sd if np.isfinite(sd) else 1.0))
        clear = min(1.0, area_px / (math.pi * (3 * rad_px) ** 2))
        area_n = min(1.0, area_px / (IMG_W * IMG_H * 0.15))
        score = W_NEAR * near + W_FLAT * flat + W_CLEAR * clear + W_AREA * area_n
        cands.append((score, cu, cv, wx, wy))
    cands.sort(reverse=True)
    return cands[0] if cands else None
