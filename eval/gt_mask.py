#!/usr/bin/env python3
"""
gt_mask.py — 절차생성 월드의 실제 3D 지오메트리로부터 GT(정답) 장애물 마스크를 계산.

손으로 라벨링하지 않는다: worlds/generate_world.py의 generate_layout(seed)이 이미
모든 오브젝트의 정확한 world 좌표/치수를 알고 있으므로, 그걸 그대로 camera.world_to_pixel()로
하강캠 이미지에 투영해서 "여기는 장애물이 있다"는 정답 마스크를 만든다.
→ 세계 정의와 정답이 항상 100% 일치함이 보장됨 (사람이 보고 마스킹하는 방식의 주관성/실수 없음).

범위와 한계 (평가표에 명시할 것):
  - GT의 "안전 여부"는 순수 기하학적 사실만 본다: 장애물 박스 위에 있는가?
  - 박스 회전(yaw)은 무시하고 axis-aligned bounding box로 보수적으로 감싼다 (실제보다
    살짝 넓게 "장애물"로 잡을 수는 있어도, 실제 장애물을 놓치지는 않음).
  - 잔디밭/도로 같은 "표면 재질 차이"는 GT 장애물이 아니다 — 그건 우리 방법(DINOv2 색상
    군집)만의 부가 안전장치라 depth-only(baseline A)/RGB-DNN(baseline B)과 공정 비교가
    안 됨. 세 방법 모두 "실제로 뭔가 위에 착륙하는가"만 동일 기준으로 비교한다.
  - 가로수/가로등(decos, layout["decos"])은 장애물에서 제외 — 배경 밀도용 시각 요소일
    뿐, 착륙에 실제로 지장을 주는 크기가 아니라고 보고 원본 데모 설계에서도 항상 무해했음.
"""
import numpy as np
from camera import IMG_W, IMG_H, world_to_pixel

# (반경 근사) 각 오브젝트 종류의 장애물 반폭(half-extent, m). 차량은 yaw를 무시하는 대신
# 대각선까지 덮도록 넉넉히 잡아 "놓침"이 없게 한다.
GARAGE_HALF = (2.3, 2.3)
HOUSE_HALF = (4.5, 4.0)
CAR_HALF = (2.3, 1.0)
TARGET_TREE_HALF = (1.8, 1.8)
PERSON_HALF = (0.3, 0.3)


def obstacle_footprints(layout):
    """generate_layout() 결과 -> [(x, y, half_x, half_y), ...] 장애물 world AABB 목록."""
    boxes = []
    for b in layout["buildings"]:
        boxes.append((b["x"], b["y"], b["sx"] / 2.0, b["sy"] / 2.0))
    for g in layout["gates"]:
        L, R = g["l"], g["r"]
        boxes.append((g["x"], L["y"], L["w"] / 2.0, L["w"] / 2.0))
        boxes.append((g["x"], R["y"], R["w"] / 2.0, R["w"] / 2.0))
    tz = layout["target_zone"]
    gx, gy = tz["garage"]
    boxes.append((gx, gy, *GARAGE_HALF))
    for h in tz["houses"]:
        boxes.append((h["x"], h["y"], *HOUSE_HALF))
    for (cx, cy, _cyaw) in tz["cars"]:
        boxes.append((cx, cy, *CAR_HALF))
    for t in tz["trees"]:
        boxes.append((t["x"], t["y"], *TARGET_TREE_HALF))
    for (px, py) in tz["persons"]:
        boxes.append((px, py, *PERSON_HALF))
    return boxes


def compute_gt_mask(layout, odom_xy, h_alt, img_w=IMG_W, img_h=IMG_H):
    """반환: (img_h, img_w) bool 배열. True = 장애물(위험), False = 안전(착륙 가능)."""
    mask = np.zeros((img_h, img_w), bool)
    for (ox, oy, hx, hy) in obstacle_footprints(layout):
        us, vs = [], []
        for dx in (-hx, hx):
            for dy in (-hy, hy):
                u, v = world_to_pixel(ox + dx, oy + dy, h_alt, odom_xy)
                us.append(u)
                vs.append(v)
        u0, u1 = int(max(0, min(us))), int(min(img_w, max(us) + 1))
        v0, v1 = int(max(0, min(vs))), int(min(img_h, max(vs) + 1))
        if u1 > u0 and v1 > v0:
            mask[v0:v1, u0:u1] = True
    return mask


def min_obstacle_distance_m(mask, u, v, h_alt, odom_xy):
    """MOD(minimum obstacle distance) — 선택한 픽셀에서 가장 가까운 장애물 픽셀까지의
    실세계 거리(m). 장애물이 화면에 하나도 없으면 None."""
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    # 픽셀 단위 유클리드 거리가 최소인 장애물 픽셀을 찾고, 그 지점의 world 좌표까지의
    # 실거리를 계산 (m/px 스케일이 화면 전역에서 거의 균일하다고 가정 — 하강캠 FOV가
    # 좁고 고도가 일정하므로 근사 오차가 작음).
    d_px = np.hypot(xs - u, ys - v)
    i = int(np.argmin(d_px))
    from camera import pixel_to_world
    ox, oy = odom_xy
    wx0, wy0 = pixel_to_world(u, v, h_alt, odom_xy)
    wx1, wy1 = pixel_to_world(float(xs[i]), float(ys[i]), h_alt, odom_xy)
    return float(np.hypot(wx1 - wx0, wy1 - wy0))
