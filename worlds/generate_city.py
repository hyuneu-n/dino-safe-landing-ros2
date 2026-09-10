#!/usr/bin/env python3
"""
generate_city.py — 대형 격자형 도시 + 드론 경로계획 (도시 확장 2단계 중 1단계)

배경 (2026-09-03): 단일 회랑(폭 80m)짜리 마을을 "테헤란로처럼 복잡한 도로망을 실제로
헤매고 다니는" 5~6배 큰 도시로 확장해달라는 요청. 2단계로 나눔:
  1단계(이 파일): 격자 도로망 + 블록별 건물 배치 + 그리드 최단경로 계산 → 월드 파일 +
                  좌표 웨이포인트(JSON)를 만든다. mission_controller.py는 아직 안 건드림.
  2단계(다음): mission_controller.py가 이 웨이포인트를 따라 실제로 회전하며 날게 하고,
              회전 중에도 기존 반응형 depth 회피(전방 카메라가 항상 진행방향을 보게 heading
              을 갱신)가 그대로 작동하게 일반화한다.

START_XY/TARGET_XY는 mission_controller.py/landing_detector.py의 기존 상수와 반드시
일치해야 하므로 바꾸지 않는다 — 대신 격자를 그 두 점이 정확히 두 모서리 노드가 되도록
설계했다. 도심 한복판(y=0 스트리트의 중간 구간)을 일부러 끊어서(BLOCKED_Y0_GAPS) —
슈퍼블록/광장이 있다고 치고 — 직선 관통이 안 되게 만들었다. 그래서 최단경로가 실제로
몇 번 꺾인다(그리드 최단경로는 Dijkstra로 계산 — 애비뉴/스트리트 간격이 균일하지
않아도 정확함).

사용법:
  python3 generate_city.py --seed 0
  python3 generate_city.py --seed 0 --mesh --out /tmp/city.world
  → /tmp/city.world 와 /tmp/city.waypoints.json (또는 --out 안 주면 generated/ 밑)
"""
import argparse
import heapq
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from generate_world import (  # noqa: E402
    box, flat_patch, mesh_house, mesh_tree, mesh_industrial, mesh_industrial_accent,
    mesh_tower, road_tile, car, cone, person, lamp_post,
    BUILDING_MESHES, TREE_MESHES, IND_MESHES, IND_ACCENT_MESHES, SKY_MESHES,
    DRONE_TEMPLATE, INTRUDER_WAIT_XY, DRONE_SPAWN_Z, validate_xml,
)

# ── mission_controller.py / landing_detector.py 와 반드시 일치 ──
START_XY = (-150.0, 0.0)
TARGET_XY = (150.0, 0.0)

# ── 격자 정의 ──
# 2026-09-03 확장: "테헤란로처럼 복잡한 도로망"(사용자가 실제 테헤란로/강남·한강대로 사진을
# 첨부하며 요청) — 애비뉴 간격을 60m→30m로 촘촘하게 늘려(6개→11개) 교차로 밀도를 약 2배로
# 올렸다. START_XY/TARGET_XY(끝 두 노드)는 그대로 두고 그 "사이"에 애비뉴를 더 끼워 넣은
# 것이라 mission_controller.py/landing_detector.py 상수는 안 건드려도 됨.
AVENUES = [-150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0]  # x, 11개
STREETS = [-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0]                              # y, 7개
ROAD_TILE_W = 14.0
BLOCK_MARGIN = 3.0   # 블록 안에서 도로 쪽으로 안 붙게 남기는 여백
# (2026-09-03: 애비뉴 간격을 60m→30m로 촘촘하게 줄이면서 8.0을 그대로 두면
#  2*(ROAD_TILE_W/2+8)=30 == 애비뉴 간격이 되어 블록 폭이 0이 되고 건물이 전혀 안 생기는
#  버그가 났었다(생성된 월드에 bld_ 모델이 0개) — 3.0으로 줄여서 애비뉴 방향으로도
#  실제 배치 폭(약 10m)이 남게 함.

# 여러 스트리트(가로 도로)에 걸쳐 구간을 끊어서 — 도심 슈퍼블록/공사/광장이 있다고 치고 —
# 최단경로가 한 번이 아니라 여러 번 꺾이며 실제 거리망을 헤매게 만드는 장치. 예전엔 중앙
# 스트리트 한 줄만 끊었더니(꺾인 횟수 3) "그냥 한 번 돌아가는" 수준이라 부족하다는 피드백
# → 인접한 스트리트에도 서로 다른 구간을 끊어서 한쪽으로 돌아가면 또 막혀 있게 했다.
# {street idx(=STREETS 인덱스): [(avenue idx, avenue idx+1), ...]}
BLOCKED_STREET_GAPS = {
    3: [(3, 4), (4, 5), (5, 6), (6, 7)],   # y=0 (중앙, 도심 광장) — 넓게 막아서 우회 강제
    2: [(6, 7), (7, 8)],                    # y=-50 — 중앙에서 위로 돌면 여기서 또 막힘
    4: [(2, 3), (3, 4)],                    # y=50  — 중앙에서 아래로 돌면 여기서 또 막힘
}
# {avenue idx: [(street idx, street idx+1), ...]} — 세로 방향(애비뉴)도 일부 끊어서 2차원으로 헤매게
BLOCKED_AVENUE_GAPS = {
    5: [(2, 3)],   # x=0 (중앙 남북대로) 일부 구간 막힘
    7: [(3, 4)],   # x=60 일부 구간 막힘
}

START_NODE = (0, 3)    # AVENUES[0]=-150, STREETS[3]=0   → START_XY와 일치
TARGET_NODE = (10, 3)  # AVENUES[10]=150, STREETS[3]=0   → TARGET_XY와 일치

# ── 맨해튼풍 보조 구역 (2026-09-04, build_manhattan_xml() 참고) — 기존 도시 동쪽에
# MANHATTAN_GAP만큼 띄운 완전히 별도의 반듯한 격자. 경로계획 그래프(build_graph)에는
# 안 들어간다 — START_NODE/TARGET_NODE와 무관.
MANHATTAN_GAP = 100.0
MANHATTAN_SPACING = 70.0
# (2026-09-04: 처음엔 32.0으로 했다가 첫 렌더 확인에서 참사가 남 — target_h를 140까지
#  올리면 SKY_MESHES 원본 비율(특히 "building-skyscraper-a"는 native h=2.88로 제일
#  납작해서 같은 target_h로 늘이면 폭이 제일 많이 뻥튀기됨) 때문에 건물 폭이 40~66m까지
#  나오는데 블록 간격이 32m라 옆 블록/도로까지 서로 뚫고 들어가 카메라가 아예 건물 속에
#  파묻히는 그림이 나왔다(스크린샷으로 확인). 70m로 넓혀서 실제 맨해튼 슈퍼블록 정도의
#  여유를 줌.
MANHATTAN_COLS, MANHATTAN_ROWS = 6, 6
MANHATTAN_X0 = max(AVENUES) + ROAD_TILE_W / 2 + MANHATTAN_GAP
MANHATTAN_AVENUES = [MANHATTAN_X0 + k * MANHATTAN_SPACING for k in range(MANHATTAN_COLS)]
_mh_half_h = (MANHATTAN_ROWS - 1) * MANHATTAN_SPACING / 2.0
MANHATTAN_STREETS = [-_mh_half_h + k * MANHATTAN_SPACING for k in range(MANHATTAN_ROWS)]

# ── 마을(비격자) 보조 구역 (2026-09-04) — "도로가 꼭 저렇게 네모반듯해야 하나, 낮은 집
# 마을도 만들어줘, 길이 반듯하지 않게" 요청. 격자(AVENUES/STREETS)도, 맨해튼 격자
# (MANHATTAN_*)도 아니고 애초에 격자 개념이 없다 — 랜덤워크로 굽이치는 폴리라인 하나(+짧은
# 골목)를 만들고 그 길을 따라 낮은 집(BUILDING_MESHES, 산업/스카이스크래퍼 없음)과 나무를
# 좌우로 흩뿌린다. 경로계획 그래프에는 안 들어간다 — 맨해튼 구역과 대칭으로 기존 도시
# 서쪽(맨해튼 반대쪽)에 배치.
VILLAGE_GAP = 100.0
VILLAGE_START_XY = (min(AVENUES) - VILLAGE_GAP, 40.0)
VILLAGE_START_HEADING = math.radians(200)  # 서남서 방향 — 기존 도시에서 계속 멀어지게


def node_xy(i, j):
    return (AVENUES[i], STREETS[j])


# ───────────────────────── 그리드 그래프 + 경로계획 ─────────────────────────

def build_graph():
    edges = {}

    def add(n1, n2, length):
        edges.setdefault(n1, []).append((n2, length))
        edges.setdefault(n2, []).append((n1, length))

    nx, ny = len(AVENUES), len(STREETS)
    for j in range(ny):
        blocked = set(BLOCKED_STREET_GAPS.get(j, []))
        for i in range(nx - 1):
            if (i, i + 1) in blocked:
                continue
            add((i, j), (i + 1, j), abs(AVENUES[i + 1] - AVENUES[i]))
    for i in range(nx):
        blocked = set(BLOCKED_AVENUE_GAPS.get(i, []))
        for j in range(ny - 1):
            if (j, j + 1) in blocked:
                continue
            add((i, j), (i, j + 1), abs(STREETS[j + 1] - STREETS[j]))
    return edges


def shortest_path(edges, start, goal):
    """Dijkstra 최단경로 (그리드 간격이 균일하지 않아서 BFS 대신 사용)."""
    dist = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]
    seen = set()
    while pq:
        d, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        if u == goal:
            break
        for v, w in edges.get(u, []):
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if goal not in dist:
        raise RuntimeError(f"{start}→{goal} 경로 없음 — 격자가 끊겨 있음 (BLOCKED_Y0_GAPS 확인)")
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def edges_on_path(path):
    return list(zip(path[:-1], path[1:]))


# ───────────────────────── 건물 배치 (블록 단위) ─────────────────────────

def district_for_block(i, j):
    """격자 중심으로부터의 거리로 구역 결정 — 도심(상업)이 가운데, 산업지대가 중간 고리,
    교외가 바깥 고리. district_for_x()의 2D 버전."""
    cx, cy = (len(AVENUES) - 1) / 2.0, (len(STREETS) - 1) / 2.0
    d = math.hypot((i + 0.5) - cx, (j + 0.5) - cy)
    dmax = math.hypot(cx, cy)
    frac = d / max(dmax, 1e-6)
    if frac < 0.35:
        return "commercial"
    if frac < 0.7:
        return "industrial"
    return "suburban"


def gen_block_buildings(rng, i, j, args):
    """블록 (i,j) 내부(애비뉴 i..i+1, 스트리트 j..j+1로 둘러싸인 사각형)에 건물을 흩뿌린다."""
    x0, x1 = AVENUES[i] + ROAD_TILE_W / 2 + BLOCK_MARGIN, AVENUES[i + 1] - ROAD_TILE_W / 2 - BLOCK_MARGIN
    y0, y1 = STREETS[j] + ROAD_TILE_W / 2 + BLOCK_MARGIN, STREETS[j + 1] - ROAD_TILE_W / 2 - BLOCK_MARGIN
    if x1 <= x0 or y1 <= y0:
        return []
    district = district_for_block(i, j)
    # 2026-09-03: 다운타운(상업)은 테헤란로/강남처럼 빽빽하게, 교외는 성기게 — 구역별로
    # 밀도를 다르게 줘야 "전체가 균일하게 복잡"이 아니라 도심만 확 밀집된 진짜 도시 느낌이 남.
    if district == "commercial":
        n = args.buildings_per_block + 4
    elif district == "industrial":
        n = args.buildings_per_block
    else:
        n = max(2, args.buildings_per_block - 3)
    out, placed = [], []
    tries = 0
    while len(out) < n and tries < n * 30:
        tries += 1
        x = rng.uniform(x0, x1)
        y = rng.uniform(y0, y1)
        sx = rng.uniform(5, 12)
        sy = rng.uniform(5, 12)
        if district == "suburban":
            h, kind, mv = rng.uniform(4, 10), "house", rng.choice(list(BUILDING_MESHES))
        elif district == "industrial":
            if rng.random() < 0.85:
                h, kind, mv = rng.uniform(6, 14), "industrial", rng.choice(list(IND_MESHES))
            else:
                h, kind, mv = rng.uniform(18, 26), "industrial_accent", rng.choice(list(IND_ACCENT_MESHES))
                sx = sy = rng.uniform(1.5, 3.0)
        else:  # commercial
            if rng.random() < 0.75:
                h, kind, mv = rng.uniform(25, args.max_height), "tower", rng.choice(list(SKY_MESHES))
            else:
                h, kind, mv = rng.uniform(4, 10), "house", rng.choice(list(BUILDING_MESHES))
        hx, hy = sx / 2, sy / 2
        if x - hx < x0 or x + hx > x1 or y - hy < y0 or y + hy > y1:
            continue
        if any(abs(x - px) < (hx + phx + 1.5) and abs(y - py) < (hy + phy + 1.5) for (px, py, phx, phy) in placed):
            continue
        placed.append((x, y, hx, hy))
        hue = rng.uniform(0.35, 0.65)
        out.append({"x": x, "y": y, "z": h / 2, "sx": sx, "sy": sy, "sz": h, "kind": kind,
                     "district": district, "mesh_variant": mv, "color": (hue, hue - 0.05, hue - 0.1)})
    return out


# ───────────────────────── 도로 렌더링 ─────────────────────────

def render_road_edge(parts, n1, n2, name_prefix):
    """도로 타일을 구간에 빈틈없이 채운다. 예전엔 정사각 타일(ROAD_TILE_W x ROAD_TILE_W)을
    등방 스케일로만 깔아서 length가 ROAD_TILE_W의 배수가 아니면 나머지만큼 사이사이가 비어
    보였다(2026-09-03 스크린샷 지적: "도로가 다 조금씩 띄어져있네"). road_tile()이 이제
    폭(cross_w)과 진행방향 길이(along_len)를 따로 받으므로, 폭은 ROAD_TILE_W로 고정하고
    진행방향 길이만 "교차로 사이 구간(usable_len)을 정확히 n등분한 몫"으로 늘여서 이어
    붙이면 교차로 가장자리부터 다음 교차로 가장자리까지 틈 없이 맞물린다."""
    (i1, j1), (i2, j2) = n1, n2
    x1, y1 = node_xy(i1, j1)
    x2, y2 = node_xy(i2, j2)
    total_len = math.hypot(x2 - x1, y2 - y1)
    yaw = 0.0 if j1 == j2 else 1.5708  # 스트리트(가로)는 0, 애비뉴(세로)는 90도
    usable_len = max(total_len - ROAD_TILE_W, ROAD_TILE_W)  # 양끝 교차로 폭만큼 제외
    n_tiles = max(1, round(usable_len / ROAD_TILE_W))
    tile_len = usable_len / n_tiles  # 나머지 없이 정확히 나눠떨어짐
    ux, uy = (x2 - x1) / total_len, (y2 - y1) / total_len
    start_d = (total_len - usable_len) / 2.0  # == ROAD_TILE_W/2, 교차로 가장자리와 정확히 맞물림
    for k in range(n_tiles):
        d = start_d + (k + 0.5) * tile_len
        x, y = x1 + ux * d, y1 + uy * d
        parts.append(road_tile(f"{name_prefix}_{k}", x, y, ROAD_TILE_W, tile_len, yaw=yaw))


def render_manhattan_road_edge(parts, x1, y1, x2, y2, yaw, name_prefix):
    """render_road_edge()와 정확히 같은 '빈틈없이 나눠떨어지는 타일링' 로직이지만, 그리드
    노드(인덱스)가 아니라 실좌표 두 점을 직접 받는다 — 아래 맨해튼 격자는 전역 AVENUES/
    STREETS(경로계획 그래프)와 완전히 분리된 별도 좌표계라 인덱스 기반 함수를 그대로
    재사용하기보다 이 쪽이 간단하다."""
    total_len = math.hypot(x2 - x1, y2 - y1)
    usable_len = max(total_len - ROAD_TILE_W, ROAD_TILE_W)
    n_tiles = max(1, round(usable_len / ROAD_TILE_W))
    tile_len = usable_len / n_tiles
    ux, uy = (x2 - x1) / total_len, (y2 - y1) / total_len
    start_d = (total_len - usable_len) / 2.0
    for k in range(n_tiles):
        d = start_d + (k + 0.5) * tile_len
        x, y = x1 + ux * d, y1 + uy * d
        parts.append(road_tile(f"{name_prefix}_{k}", x, y, ROAD_TILE_W, tile_len, yaw=yaw))


def gen_manhattan_block_buildings(rng, x0, x1, y0, y1):
    """맨해튼 블록 하나 — 산업/교외 메시는 아예 안 쓰고 SKY_MESHES(스카이스크래퍼)만, 기존
    상업지구보다도 더 빽빽하게(건물 사이 여백 1.5→0.8m) 채운다. 색조도 hue를 좁은 범위
    (0.5~0.62)로 고정해 유리 파사드 느낌의 청회색 계열로 통일 — 기존 상업지구(hue 0.35~0.65
    랜덤이라 갈색~녹색까지 섞임)와 시각적으로 확실히 구분되게."""
    x0, x1 = x0 + ROAD_TILE_W / 2 + BLOCK_MARGIN, x1 - ROAD_TILE_W / 2 - BLOCK_MARGIN
    y0, y1 = y0 + ROAD_TILE_W / 2 + BLOCK_MARGIN, y1 - ROAD_TILE_W / 2 - BLOCK_MARGIN
    if x1 <= x0 or y1 <= y0:
        return []
    # 2026-09-04: mesh_tower()는 target_h/native_h로 등방 스케일하기 때문에, 목표 높이가
    # 크면 원본 비율 그대로 폭도 같이 커진다(특히 "building-skyscraper-a"는 native h=2.88로
    # 제일 납작해서 같은 target_h를 줘도 폭이 제일 많이 뻥튀기됨) — 실측: h=140이면 폭이
    # 40~66m까지 나와서 70m 블록도 넘길 뻔했다. n을 10→4로 줄이고(실제 맨해튼도 블록 하나에
    # 큰 건물 1~4채 정도) 높이를 50~110으로 낮춰 폭을 30~48m 선으로 눌렀다.
    n = 4
    out, placed = [], []
    tries = 0
    while len(out) < n and tries < n * 40:
        tries += 1
        x = rng.uniform(x0, x1)
        y = rng.uniform(y0, y1)
        h = rng.uniform(50, 110)
        mv = rng.choice(list(SKY_MESHES))
        sx = rng.uniform(14, 20)
        sy = rng.uniform(14, 20)
        hx, hy = sx / 2, sy / 2
        if x - hx < x0 or x + hx > x1 or y - hy < y0 or y + hy > y1:
            continue
        if any(abs(x - px) < (hx + phx + 2.0) and abs(y - py) < (hy + phy + 2.0) for (px, py, phx, phy) in placed):
            continue
        placed.append((x, y, hx, hy))
        hue = rng.uniform(0.5, 0.62)
        out.append({"x": x, "y": y, "z": h / 2, "sx": sx, "sy": sy, "sz": h,
                     "mesh_variant": mv, "color": (hue - 0.15, hue - 0.1, hue)})
    return out


def build_manhattan_xml(parts, rng, args, bcount_start):
    """2026-09-04: "여기도 좋은데 공사장 느낌이다 — 진짜 뉴욕시티 같은 도시를 옆에 하나 더
    만들어줘(기존 건 지우지 말고), 목적지를 다양하게 설정하려고" 라는 요청에 대응. 기존 격자
    (도심 타워/산업지대 탱크·굴뚝/교외 주택이 섞인 지구)는 전혀 안 건드리고, 그 동쪽으로
    MANHATTAN_GAP만큼 띄워서 완전히 별도의 격자를 하나 더 붙인다. 기존 지구와의 차이:
      - 산업/교외 메시(IND_MESHES/BUILDING_MESHES)를 아예 안 씀 → 탱크/굴뚝처럼 "공사장"
        느낌을 주는 오브젝트가 하나도 안 나옴, 전부 SKY_MESHES(스카이스크래퍼)만.
      - 우회를 강제하려고 일부러 끊어놓은 기존 그리드(BLOCKED_*_GAPS)와 달리 완전히 반듯한
        격자(끊긴 구간 없음) — 실제 맨해튼처럼 애비뉴/스트리트가 쭉 뻗어있다.
      - START_XY→TARGET_XY 최단경로(Dijkstra) 그래프에는 안 넣는다 — mission_controller.py의
        기존 웨이포인트 추종 로직은 손 안 대고, 순전히 "클릭으로 목적지를 옮길 수 있는 또
        다른 도시"로만 존재한다(다양한 착륙지 실험용).
    자체 잔디 바닥(기존 ground_cover와 안 겹치는 별도 패치)까지 포함해서 붕 뜬 느낌이
    안 나게 한다."""
    xs, ys = MANHATTAN_AVENUES, MANHATTAN_STREETS
    gx_min, gx_max = min(xs) - 40, max(xs) + 40
    gy_min, gy_max = min(ys) - 40, max(ys) + 40
    g_len, g_w = (gx_max - gx_min), (gy_max - gy_min)
    gcx, gcy = (gx_min + gx_max) / 2, (gy_min + gy_max) / 2
    parts.append('\n    <!-- 맨해튼풍 보조 구역 (다양한 목적지용 — 기존 도시와 완전히 별개, 경로계획 그래프 밖) -->\n')
    parts.append('    <model name="ground_cover_manhattan"><static>true</static>'
                  f'<pose>{gcx:.1f} {gcy:.1f} 0.001 0 0 0</pose><link name="l">'
                  f'<visual name="v"><geometry><box><size>{g_len:.1f} {g_w:.1f} 0.01</size></box></geometry>'
                  '<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
                  '<name>Gazebo/Grass</name></script></material></visual></link></model>\n\n')

    for j, y in enumerate(ys):
        for i in range(len(xs) - 1):
            render_manhattan_road_edge(parts, xs[i], y, xs[i + 1], y, 0.0, f"mh_road_h{i}_{j}")
    for i, x in enumerate(xs):
        for j in range(len(ys) - 1):
            render_manhattan_road_edge(parts, x, ys[j], x, ys[j + 1], 1.5708, f"mh_road_v{i}_{j}")
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            if args.mesh:
                uri = f"file://{os.path.join(HERE, 'meshes', 'kenney_city_kit_roads_dae')}/road-crossroad.dae"
                scale = ROAD_TILE_W / 1.0
                parts.append(f'<model name="mh_isect_{i}_{j}"><static>true</static>'
                              f'<pose>{x:.2f} {y:.2f} 0 0 0 0</pose><link name="l">'
                              f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
                              f'<geometry><mesh><uri>{uri}</uri>'
                              f'<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale></mesh></geometry></visual>'
                              f'</link></model>\n')
            else:
                parts.append(flat_patch(f"mh_isect_{i}_{j}", x, y, ROAD_TILE_W, ROAD_TILE_W,
                                         (0.28, 0.28, 0.30), z=0.005))

    bcount = bcount_start
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            blds = gen_manhattan_block_buildings(rng, xs[i], xs[i + 1], ys[j], ys[j + 1])
            for b in blds:
                name = f"bld_mh_{bcount:04d}"
                bcount += 1
                if args.mesh:
                    parts.append(mesh_tower(name, b["x"], b["y"], b["mesh_variant"], target_h=b["sz"],
                                             collision_w=b["sx"], collision_d=b["sy"]))
                else:
                    parts.append(box(name, b["x"], b["y"], b["z"], b["sx"], b["sy"], b["sz"], color=b["color"]))
    return bcount, (gx_min, gx_max, gy_min, gy_max)


def generate_winding_path(rng, start_xy, start_heading, n_segments,
                           seg_len_range=(10.0, 18.0), turn_range_deg=(-26, 26)):
    """랜덤워크로 굽이치는 폴리라인 — 매 구간마다 이전 heading에서 조금씩만(turn_range_deg)
    꺾어서 급커브 없이 자연스럽게 휘어지는 '시골길' 느낌을 낸다. 격자 도로처럼 노드 인덱스가
    아니라 실좌표 점 목록을 그대로 반환한다."""
    pts = [start_xy]
    heading = start_heading
    x, y = start_xy
    for _ in range(n_segments):
        heading += math.radians(rng.uniform(*turn_range_deg))
        seg_len = rng.uniform(*seg_len_range)
        x, y = x + math.cos(heading) * seg_len, y + math.sin(heading) * seg_len
        pts.append((x, y))
    return pts


def generate_connector_path(rng, start_xy, end_xy, seg_len=14.0, jitter_deg=16):
    """2026-09-04: "도시랑 마을을 연결은 해야하지 않겠음??" 요청 — 시작점에서 목표점 쪽으로
    매 구간 목표 방향을 다시 계산해서(steering) 조금씩 지터를 주며 나아가다가, 남은 거리가
    한 구간보다 짧아지면 마지막 점을 목표에 정확히 스냅한다. generate_winding_path()의
    순수 랜덤워크와 달리 반드시 end_xy에 도달하는 게 보장된다 — 도시 교차로와 마을 진입점을
    실제로 이어야 해서 "거의 도착"으로는 안 됨."""
    pts = [start_xy]
    x, y = start_xy
    tx, ty = end_xy
    guard = 0
    while math.hypot(tx - x, ty - y) > seg_len * 1.3 and guard < 200:
        guard += 1
        desired = math.atan2(ty - y, tx - x)
        heading = desired + math.radians(rng.uniform(-jitter_deg, jitter_deg))
        x, y = x + math.cos(heading) * seg_len, y + math.sin(heading) * seg_len
        pts.append((x, y))
    pts.append(end_xy)
    return pts


def render_village_path(parts, pts, name_prefix, road_w=6.0):
    """폴리라인의 구간마다 road_tile 하나씩(그 구간의 실제 길이·방향에 맞춰) 이어붙인다.
    render_road_edge()와 달리 '정확히 나눠떨어지는 배수'를 신경 쓸 필요가 없다 — 구간 하나당
    타일 하나라 애초에 나머지가 안 생긴다(각 구간이 이미 정확히 그 길이의 타일 하나니까)."""
    for k in range(len(pts) - 1):
        x1, y1 = pts[k]
        x2, y2 = pts[k + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < 0.5:
            continue
        yaw = math.atan2(y2 - y1, x2 - x1)
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        parts.append(road_tile(f"{name_prefix}_{k}", mx, my, road_w, seg_len, yaw=yaw))


def _point_on_path(pts, seg_lens, total_len, d):
    """경로 위 임의의 누적거리 d에 해당하는 (x, y, heading) 반환."""
    acc = 0.0
    for k, sl in enumerate(seg_lens):
        if d <= acc + sl or k == len(seg_lens) - 1:
            t = (d - acc) / sl if sl > 1e-6 else 0.0
            x1, y1 = pts[k]
            x2, y2 = pts[k + 1]
            return x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, math.atan2(y2 - y1, x2 - x1)
        acc += sl
    x1, y1 = pts[-2]
    x2, y2 = pts[-1]
    return x2, y2, math.atan2(y2 - y1, x2 - x1)


#  2026-09-04: "집을 더 빽빽하게! 집 색상은 바꿀 수 없는거지?" 요청 — 색상 테스트 월드
#  (down_cam 캡처로 확인)로 SDF <material>이 메시 텍스처를 완전히 덮어쓴다는 걸 확인했으니,
#  집집마다 다른 색을 준다. 너무 알록달록하지 않게(원색 대신) 실제 시골 마을 벽/지붕에
#  흔한 채도 낮은 색 위주로 팔레트를 잡았다.
VILLAGE_HOUSE_COLORS = [
    (0.80, 0.58, 0.36),  # 테라코타
    (0.86, 0.80, 0.62),  # 크림
    (0.55, 0.63, 0.45),  # 세이지그린
    (0.46, 0.52, 0.60),  # 슬레이트블루
    (0.78, 0.62, 0.24),  # 머스타드
    (0.64, 0.40, 0.32),  # 벽돌
    (0.88, 0.88, 0.86),  # 화이트워시
]


def gen_village_scatter(rng, pts, road_w, n_houses, n_trees, house_margin=1.2):
    """길을 따라 좌우로 낮은 집/나무를 흩뿌린다 — 격자 블록이 아니라 경로상 임의 지점에서
    수직 방향으로 옵셋을 줘서 배치하므로 정렬이 없다(=마을 골목길 느낌). 집과 나무를 같은
    placed 리스트로 겹침 검사해서 나무가 집 마당 한복판에 박히는 일이 없게 한다.
    house_margin(2026-09-04, "집을 더 빽빽하게" 요청): 기본 2.0→1.2로 줄여서 같은 구간에
    더 많은 집이 들어가게 함(overlaps() 판정 여유를 줄이는 것 = 밀도를 올리는 것)."""
    seg_lens = [math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]) for k in range(len(pts) - 1)]
    total_len = sum(seg_lens)
    houses, trees, placed = [], [], []

    tries = 0
    while len(houses) < n_houses and tries < n_houses * 80:
        tries += 1
        px, py, heading = _point_on_path(pts, seg_lens, total_len, rng.uniform(0, total_len))
        side = rng.choice([-1, 1])
        offset = rng.uniform(road_w / 2 + 3.5, road_w / 2 + 10)
        perp = heading + math.pi / 2
        hx, hy = px + math.cos(perp) * offset * side, py + math.sin(perp) * offset * side
        w = rng.uniform(5, 9)
        if any(abs(hx - ox) < (w / 2 + ow / 2 + house_margin) and abs(hy - oy) < (w / 2 + ow / 2 + house_margin)
               for (ox, oy, ow) in placed):
            continue
        placed.append((hx, hy, w))
        yaw = heading + (0 if side > 0 else math.pi) + rng.uniform(-0.35, 0.35)
        houses.append({"x": hx, "y": hy, "yaw": yaw, "w": w, "mesh_variant": rng.choice(list(BUILDING_MESHES)),
                        "color": rng.choice(VILLAGE_HOUSE_COLORS)})

    tries = 0
    while len(trees) < n_trees and tries < n_trees * 60:
        tries += 1
        px, py, heading = _point_on_path(pts, seg_lens, total_len, rng.uniform(0, total_len))
        side = rng.choice([-1, 1])
        offset = rng.uniform(road_w / 2 + 2, road_w / 2 + 16)
        perp = heading + math.pi / 2
        tx, ty = px + math.cos(perp) * offset * side, py + math.sin(perp) * offset * side
        r = 1.4
        if any(abs(tx - ox) < (r + ow / 2 + 1.0) and abs(ty - oy) < (r + ow / 2 + 1.0) for (ox, oy, ow) in placed):
            continue
        placed.append((tx, ty, r * 2))
        trees.append({"x": tx, "y": ty, "mesh_variant": rng.choice(list(TREE_MESHES))})

    return houses, trees


def build_village_xml(parts, rng, args):
    """2026-09-04: "도로가 꼭 저렇게 네모반듯해야 하나, 낮은 집 마을도 만들어줘 — 길이
    반듯하지 않게" 요청. 기존 격자도시/맨해튼 구역은 안 건드리고, 그 서쪽에(맨해튼과 대칭)
    격자 개념 자체가 없는 마을을 하나 더 붙인다: 랜덤워크 폴리라인 메인 길 하나 + 짧은
    골목 2개, 그 옆으로 낮은 집(BUILDING_MESHES)과 나무를 비정렬로 흩뿌림. 경로계획
    그래프에는 안 들어간다 — 순전히 시각적/클릭 리타겟용 세 번째 목적지 후보."""
    main_pts = generate_winding_path(rng, VILLAGE_START_XY, VILLAGE_START_HEADING, n_segments=22)
    all_pts = list(main_pts)
    parts.append('\n    <!-- 마을 보조 구역 (비격자, 낮은 집 — 기존 도시/맨해튼과 완전히 별개) -->\n')
    render_village_path(parts, main_pts, "village_road_main", road_w=6.0)

    # 2026-09-04: "도시랑 마을을 연결은 해야하지 않겠음??" — 드론 출발점이자 서쪽 애비뉴의
    # 실제 교차로인 START_XY(=START_NODE의 좌표, 이미 render_intersection으로 크로스로드
    # 메시가 깔려 있는 자리)에서 마을 진입점(VILLAGE_START_XY)까지 시골길로 잇는다.
    # generate_winding_path()의 순수 랜덤워크는 도착점 도달을 보장 안 해서 반드시 이어야
    # 하는 이 구간엔 generate_connector_path()(steering + 목표 스냅)를 쓴다.
    connector_pts = generate_connector_path(rng, START_XY, VILLAGE_START_XY)
    render_village_path(parts, connector_pts, "village_connector", road_w=6.0)
    all_pts.extend(connector_pts)

    branches = []
    for b in range(2):
        anchor_idx = rng.randint(3, len(main_pts) - 4)
        ax, ay = main_pts[anchor_idx]
        base_heading = math.atan2(main_pts[anchor_idx + 1][1] - main_pts[anchor_idx][1],
                                   main_pts[anchor_idx + 1][0] - main_pts[anchor_idx][0])
        branch_heading = base_heading + rng.choice([-1, 1]) * math.radians(rng.uniform(70, 110))
        branch_pts = generate_winding_path(rng, (ax, ay), branch_heading, n_segments=8,
                                            seg_len_range=(8.0, 14.0), turn_range_deg=(-20, 20))
        render_village_path(parts, branch_pts, f"village_road_branch{b}", road_w=5.0)
        branches.append(branch_pts)
        all_pts.extend(branch_pts)

    # 2026-09-10: "마을이 여전히 부실하다, 마을 같지가 않다" — 밀도 더 올리고(34→46, 11→14),
    # 중심 광장(우물+마을회관+집 링)을 넣어 "중심이 있는 정착지"로 만든다.
    houses, trees = gen_village_scatter(rng, main_pts, 6.0, n_houses=46, n_trees=26)
    for b_pts in branches:
        bh, bt = gen_village_scatter(rng, b_pts, 5.0, n_houses=14, n_trees=7)
        houses.extend(bh)
        trees.extend(bt)
    ch, ct = gen_village_scatter(rng, connector_pts, 6.0, n_houses=10, n_trees=7)
    houses.extend(ch)
    trees.extend(ct)

    # 중심 광장 — 메인 길 55% 지점 옆으로 살짝 비켜서
    sq_seg_lens = [math.hypot(main_pts[k + 1][0] - main_pts[k][0], main_pts[k + 1][1] - main_pts[k][1])
                   for k in range(len(main_pts) - 1)]
    sq_px, sq_py, sq_head = _point_on_path(main_pts, sq_seg_lens, sum(sq_seg_lens), sum(sq_seg_lens) * 0.55)
    sq_cx = sq_px + math.cos(sq_head + math.pi / 2) * 34.0
    sq_cy = sq_py + math.sin(sq_head + math.pi / 2) * 34.0
    build_village_square(parts, rng, sq_cx, sq_cy)

    # 마을 안 배송 목적지 2곳 (랜덤워크 형상에 맞춰 생성)
    end_x, end_y = main_pts[-1]
    village_spots = [
        {"x": round(sq_cx, 1), "y": round(sq_cy, 1), "label": "마을 회관 앞마당", "kind": "green",
         "note": "우물·벤치·집으로 둘러싸인 마을 중심 — 개방됐지만 사방이 저층 건물"},
        {"x": round(end_x, 1), "y": round(end_y, 1), "label": "마을 어귀 공터", "kind": "vacant",
         "note": "마을 끝 비포장 공터 — 도심과 완전히 다른 시골 배경"},
    ]

    bcount = 0
    for h in houses:
        name = f"bld_vil_{bcount:04d}"
        bcount += 1
        if args.mesh:
            parts.append(mesh_house(name, h["x"], h["y"], h["mesh_variant"], target_w=h["w"], yaw=h["yaw"],
                                     color=h["color"]))
        else:
            parts.append(box(name, h["x"], h["y"], 3.0, h["w"], h["w"] * 0.8, 6.0,
                              yaw=h["yaw"], color=h["color"]))
    tcount = 0
    for t in trees:
        name = f"tree_vil_{tcount:04d}"
        tcount += 1
        if args.mesh:
            parts.append(mesh_tree(name, t["x"], t["y"], t["mesh_variant"], target_h=4.5))

    xs_all = [p[0] for p in all_pts]
    ys_all = [p[1] for p in all_pts]
    gx_min, gx_max = min(xs_all) - 25, max(xs_all) + 25
    gy_min, gy_max = min(ys_all) - 25, max(ys_all) + 25
    g_len, g_w = gx_max - gx_min, gy_max - gy_min
    gcx, gcy = (gx_min + gx_max) / 2, (gy_min + gy_max) / 2
    parts.append('    <model name="ground_cover_village"><static>true</static>'
                  f'<pose>{gcx:.1f} {gcy:.1f} 0.001 0 0 0</pose><link name="l">'
                  f'<visual name="v"><geometry><box><size>{g_len:.1f} {g_w:.1f} 0.01</size></box></geometry>'
                  '<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
                  '<name>Gazebo/Grass</name></script></material></visual></link></model>\n\n')
    return (gx_min, gx_max, gy_min, gy_max), village_spots


def render_intersection(parts, node, args):
    i, j = node
    x, y = node_xy(i, j)
    if args.mesh:
        uri = f"file://{os.path.join(HERE, 'meshes', 'kenney_city_kit_roads_dae')}/road-crossroad.dae"
        scale = ROAD_TILE_W / 1.0
        parts.append(f'<model name="isect_{i}_{j}"><static>true</static>'
                      f'<pose>{x:.2f} {y:.2f} 0 0 0 0</pose><link name="l">'
                      f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
                      f'<geometry><mesh><uri>{uri}</uri>'
                      f'<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale></mesh></geometry></visual>'
                      f'</link></model>\n')
    else:
        parts.append(flat_patch(f"isect_{i}_{j}", x, y, ROAD_TILE_W, ROAD_TILE_W,
                                 (0.28, 0.28, 0.30), z=0.005))


# ───────────────────────── 배송 목적지 (클릭 리타겟용 착륙 시나리오) ─────────────────────────
# 2026-09-10: "목적지가 많지 않고 다 똑같이 비행한다, 사람들이 누를 맵이 없다" 요청.
# 성격이 다른 착륙 시나리오를 맵 곳곳에 만들고(광장/공원/주차장/공터/협곡/옥상/마을회관),
# 각각 이름표 깃발 마커로 노출한다. mission_controller.py는 이미 /clicked_point 리타겟을
# 지원하므로, 여기서는 (1) 시각적으로 구분되는 착륙지 피처 + (2) 발견 가능한 마커 +
# (3) waypoints.json에 좌표·라벨을 실어주면 된다. START_XY→TARGET_XY 경로계획은 안 건드림.

def _mat(color, a=1.0):
    r, g, b = color
    return (f'<material><ambient>{r:.2f} {g:.2f} {b:.2f} {a}</ambient>'
            f'<diffuse>{min(r+0.05,1):.2f} {min(g+0.05,1):.2f} {min(b+0.05,1):.2f} {a}</diffuse></material>')


def ground_patch(name, x, y, sx, sy, color, z=0.02):
    """바닥 패치(포장/흙/잔디 등). generate_world.flat_patch()는 <ambient>만 있어서 밝은 색이
    햇빛에 하얗게 날아간다(2026-09-10 렌더 확인) — box()는 ambient+diffuse를 둘 다 써서 색이
    제대로 나오므로 얇은 무충돌 박스로 깐다."""
    return box(name, x, y, z, sx, sy, 0.04, color=color, collide=False)


def landing_flag(parts, name, x, y, color, label_h=6.5):
    """착륙지 위치 표시용 깃발 — 흰 기둥 + 색깔 깃발 + 꼭대기 구슬. 사람이 RViz/Gazebo에서
    "저기가 후보구나" 하고 알아볼 수 있게 밝고 크게. 사람 읽는 라벨은 waypoints.json으로."""
    parts.append(
        f'<model name="{name}_flag"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose><link name="l">'
        f'<visual name="pole"><pose>0 0 {label_h/2:.2f} 0 0 0</pose>'
        f'<geometry><cylinder><radius>0.13</radius><length>{label_h:.2f}</length></cylinder></geometry>{_mat((0.93,0.93,0.93))}</visual>'
        f'<visual name="flag"><pose>0.85 0 {label_h-0.65:.2f} 0 0 0</pose>'
        f'<geometry><box><size>1.7 0.07 1.15</size></box></geometry>{_mat(color)}</visual>'
        f'<visual name="ball"><pose>0 0 {label_h+0.18:.2f} 0 0 0</pose>'
        f'<geometry><sphere><radius>0.32</radius></sphere></geometry>{_mat(color)}</visual>'
        f'</link></model>\n')


def feat_helipad(parts, name, x, y, size=9.0):
    parts.append(ground_patch(f"{name}_pad", x, y, size, size, (0.30, 0.30, 0.33)))
    s = size
    parts.append(
        f'<model name="{name}_H"><static>true</static><pose>{x:.2f} {y:.2f} 0.035 0 0 0</pose><link name="l">'
        f'<visual name="a"><pose>{-s*0.16:.2f} 0 0 0 0 0</pose><geometry><box><size>{s*0.12:.2f} {s*0.5:.2f} 0.02</size></box></geometry>{_mat((0.96,0.96,0.96))}</visual>'
        f'<visual name="b"><pose>{s*0.16:.2f} 0 0 0 0 0</pose><geometry><box><size>{s*0.12:.2f} {s*0.5:.2f} 0.02</size></box></geometry>{_mat((0.96,0.96,0.96))}</visual>'
        f'<visual name="c"><pose>0 0 0 0 0 0</pose><geometry><box><size>{s*0.32:.2f} {s*0.12:.2f} 0.02</size></box></geometry>{_mat((0.96,0.96,0.96))}</visual>'
        f'</link></model>\n')


def feat_plaza(parts, name, x, y, rng, size=16.0):
    """포장 광장 — 가로등 4개 + 화단 박스 몇 개로 둘러싸고 중앙만 비움 (주변 장애물 많음)."""
    parts.append(ground_patch(f"{name}_pave", x, y, size, size, (0.55, 0.52, 0.48)))
    h = size / 2 - 1.0
    for k, (dx, dy) in enumerate([(-h, -h), (h, -h), (-h, h), (h, h)]):
        parts.append(lamp_post(f"{name}_lamp{k}", x + dx, y + dy))
    for k in range(5):
        px = x + rng.uniform(-h, h)
        py = y + rng.choice([-1, 1]) * rng.uniform(h * 0.55, h)
        parts.append(box(f"{name}_planter{k}", px, py, 0.4, rng.uniform(1.6, 2.6), rng.uniform(1.0, 1.6), 0.8,
                          color=(0.30, 0.45, 0.28)))


def feat_park(parts, name, x, y, rng, r=13.0):
    """근린공원 — 가장자리 나무 링 + 벤치 몇 개, 중앙만 개방 (반경 안에서 골라야 함)."""
    n = 12
    for k in range(n):
        a = 2 * math.pi * k / n + rng.uniform(-0.15, 0.15)
        tx, ty = x + math.cos(a) * r, y + math.sin(a) * r
        parts.append(mesh_tree(f"{name}_tree{k}", tx, ty, rng.choice(list(TREE_MESHES)), target_h=rng.uniform(4.0, 6.0)))
    for k in range(3):
        a = rng.uniform(0, 2 * math.pi)
        bx, by = x + math.cos(a) * (r * 0.45), y + math.sin(a) * (r * 0.45)
        parts.append(box(f"{name}_bench{k}", bx, by, 0.35, 2.4, 0.6, 0.7, yaw=a, color=(0.45, 0.32, 0.2)))


def feat_parking(parts, name, x, y, rng, w=24.0, d=17.0):
    """마트 주차장 — 아스팔트 + 차량 그리드, 중앙 근처 스톨 하나만 비움 (난이도 상)."""
    parts.append(ground_patch(f"{name}_asph", x, y, w, d, (0.22, 0.22, 0.24)))
    cols = max(int(w // 6), 2)
    rows = max(int(d // 5), 2)
    empties = {(cols // 2, rows // 2), (cols // 2 - 1, rows // 2)}
    palette = [(0.75, 0.1, 0.1), (0.1, 0.2, 0.6), (0.85, 0.85, 0.85), (0.1, 0.1, 0.12), (0.6, 0.55, 0.1)]
    for ci in range(cols):
        for ri in range(rows):
            if (ci, ri) in empties:
                continue
            cx = x - w / 2 + 3 + ci * (w - 6) / max(cols - 1, 1)
            cy = y - d / 2 + 2.5 + ri * (d - 5) / max(rows - 1, 1)
            parts.append(car(f"{name}_car{ci}_{ri}", cx, cy, 1.5708, rng.choice(palette)))


def feat_vacant(parts, name, x, y, rng, w=20.0, d=16.0):
    """공터/야적장 — 비포장 흙바닥 + 잔해 박스·콘·부분 펜스 (표면이 애매, semantic 배제 테스트)."""
    parts.append(ground_patch(f"{name}_dirt", x, y, w, d, (0.42, 0.34, 0.24)))
    for k in range(6):
        px, py = x + rng.uniform(-w / 2 + 2, w / 2 - 2), y + rng.uniform(-d / 2 + 2, d / 2 - 2)
        if math.hypot(px - x, py - y) < 3.0:
            continue
        parts.append(box(f"{name}_debris{k}", px, py, rng.uniform(0.3, 0.8), rng.uniform(1.0, 2.5),
                          rng.uniform(1.0, 2.0), rng.uniform(0.6, 1.6), yaw=rng.uniform(0, 3.14),
                          color=(0.5, 0.48, 0.45)))
    for k in range(4):
        parts.append(cone(f"{name}_cone{k}", x + rng.uniform(-w / 2, w / 2), y + rng.uniform(-d / 2, d / 2)))
    for k in range(5):  # 한쪽 펜스
        parts.append(box(f"{name}_fence{k}", x - w / 2, y - d / 2 + 1.5 + k * (d - 3) / 4, 0.9,
                          0.15, (d - 3) / 4 * 0.9, 1.8, color=(0.55, 0.55, 0.58)))


def feat_rooftop(parts, name, x, y, bh=6.5, size=11.0):
    """옥상 패드 — 지상보다 높은 포장 플랫폼(주차타워 옥상 느낌). 상면이 좁고 가장자리 낙하
    위험. 지상 하강 로직으로도 대충 되도록 높이는 낮게(6~7m)."""
    parts.append(box(f"{name}_podium", x, y, bh / 2, size, size, bh, color=(0.5, 0.5, 0.54)))
    parts.append(ground_patch(f"{name}_deck", x, y, size * 0.92, size * 0.92, (0.32, 0.32, 0.36), z=bh + 0.02))
    parts.append(
        f'<model name="{name}_H"><static>true</static><pose>{x:.2f} {y:.2f} {bh+0.05:.2f} 0 0 0</pose><link name="l">'
        f'<visual name="a"><pose>-1.6 0 0 0 0 0</pose><geometry><box><size>1.0 4.0 0.03</size></box></geometry>{_mat((0.96,0.96,0.96))}</visual>'
        f'<visual name="b"><pose>1.6 0 0 0 0 0</pose><geometry><box><size>1.0 4.0 0.03</size></box></geometry>{_mat((0.96,0.96,0.96))}</visual>'
        f'<visual name="c"><pose>0 0 0 0 0 0</pose><geometry><box><size>2.2 1.0 0.03</size></box></geometry>{_mat((0.96,0.96,0.96))}</visual>'
        f'</link></model>\n')


_SPOT_FLAG_COLORS = {
    "helipad": (0.1, 0.8, 0.25), "plaza": (0.95, 0.75, 0.1), "park": (0.2, 0.7, 0.35),
    "parking": (0.9, 0.4, 0.1), "vacant": (0.7, 0.5, 0.3), "street": (0.85, 0.15, 0.5),
    "rooftop": (0.2, 0.55, 0.95), "green": (0.35, 0.75, 0.4),
}


def landing_spots_static():
    """맵에 고정 배치되는 배송 목적지 후보들. 마을 안 목적지 2개는 build_village_xml()이
    (랜덤워크로 생성되는 마을 형상에 맞춰) 따로 만들어 붙인다."""
    mcx = sum(MANHATTAN_AVENUES) / len(MANHATTAN_AVENUES)
    mcy = sum(MANHATTAN_STREETS) / len(MANHATTAN_STREETS)
    return [
        {"x": TARGET_XY[0], "y": TARGET_XY[1], "label": "배송 목적지 (기본 경로 종점)", "kind": "helipad",
         "note": "격자도시 동쪽 끝, Dijkstra 경로가 실제 도착하는 지점"},
        {"x": 182.0, "y": -45.0, "label": "도심 광장", "kind": "plaza",
         "note": "가로등·화단으로 둘러싸인 포장 광장 — 주변 장애물 많음"},
        {"x": -188.0, "y": 92.0, "label": "근린공원", "kind": "park",
         "note": "가장자리 나무 링, 중앙만 개방 — 반경 안에서 골라야 함"},
        {"x": 128.0, "y": 182.0, "label": "대형마트 주차장", "kind": "parking",
         "note": "빽빽한 차량 사이 빈 스톨 하나 — 난이도 상"},
        {"x": -178.0, "y": -168.0, "label": "산업단지 공터", "kind": "vacant",
         "note": "잔해·펜스 있는 비포장 부지 — 표면이 애매(semantic 배제 테스트)"},
        {"x": MANHATTAN_AVENUES[1], "y": (MANHATTAN_STREETS[2] + MANHATTAN_STREETS[3]) / 2,
         "label": "맨해튼 스트리트 협곡", "kind": "street",
         "note": "고층 사이 좁은 도로 — 하강 중 측면 여유 거의 없음"},
        {"x": MANHATTAN_AVENUES[4], "y": MANHATTAN_STREETS[4], "label": "고층빌딩 옥상 패드", "kind": "rooftop",
         "note": "고층 사이 교차로에 세운 포장 플랫폼(주차타워 옥상 느낌) — 상면이 좁고 가장자리 낙하 위험"},
    ]


def render_landing_spot(parts, name, spot, rng):
    x, y, kind = spot["x"], spot["y"], spot["kind"]
    if kind == "helipad":
        feat_helipad(parts, name, x, y)
    elif kind == "plaza":
        feat_plaza(parts, name, x, y, rng)
    elif kind == "park":
        feat_park(parts, name, x, y, rng)
    elif kind == "parking":
        feat_parking(parts, name, x, y, rng)
    elif kind == "vacant":
        feat_vacant(parts, name, x, y, rng)
    elif kind == "rooftop":
        feat_rooftop(parts, name, x, y, bh=spot.get("bh", 18.0))
    elif kind == "green":
        feat_helipad(parts, name, x, y, size=8.0)  # 마을 회관 앞 포장 마당
    # "street"는 협곡 자체가 시나리오라 별도 피처 없음
    landing_flag(parts, name, x, y, _SPOT_FLAG_COLORS.get(kind, (0.9, 0.9, 0.1)))


def build_village_square(parts, rng, cx, cy):
    """마을 중심 광장 — 우물 + 마을회관(첨탑) + 벤치 + 촘촘한 집 링. "마을 같지가 않다"는
    피드백 대응: 길가에 집만 흩어져 있던 걸 "중심이 있는 정착지"로 만든다."""
    parts.append(ground_patch("vsq_ground", cx, cy, 34.0, 34.0, (0.50, 0.44, 0.30)))  # 다져진 흙 마당
    parts.append(f'<model name="vsq_well"><static>true</static><pose>{cx:.2f} {cy:.2f} 0 0 0 0</pose><link name="l">'
                  f'<visual name="w"><pose>0 0 0.5 0 0 0</pose><geometry><cylinder><radius>1.1</radius><length>1.0</length></cylinder></geometry>'
                  f'{_mat((0.5,0.5,0.52))}</visual>'
                  f'<visual name="roof"><pose>0 0 2.4 0 0 0</pose><geometry><box><size>2.6 2.6 0.2</size></box></geometry>'
                  f'{_mat((0.4,0.25,0.15))}</visual></link></model>\n')
    hx, hy = cx + 15.0, cy
    parts.append(box("vsq_hall", hx, hy, 3.0, 9.0, 7.0, 6.0, color=(0.85, 0.82, 0.7)))
    parts.append(box("vsq_spire", hx, hy + 2.0, 8.0, 1.2, 1.2, 4.0, color=(0.55, 0.15, 0.15)))
    for k, (dx, dy, yw) in enumerate([(-7, 0, 0), (7, 0, 0), (0, -7, 1.5708), (0, 7, 1.5708)]):
        parts.append(box(f"vsq_bench{k}", cx + dx, cy + dy, 0.35, 2.6, 0.6, 0.7, yaw=yw, color=(0.45, 0.32, 0.2)))
    for k in range(9):
        a = 2 * math.pi * k / 9 + rng.uniform(-0.15, 0.15)
        r = rng.uniform(22.0, 26.0)
        px, py = cx + math.cos(a) * r, cy + math.sin(a) * r
        parts.append(mesh_house(f"bld_vsq_{k}", px, py, rng.choice(list(BUILDING_MESHES)),
                                 target_w=rng.uniform(5.5, 8.0), yaw=a + math.pi,
                                 color=rng.choice(VILLAGE_HOUSE_COLORS)))


# ───────────────────────── 조립 ─────────────────────────

def build_city_xml(args, seed):
    rng = random.Random(seed)
    edges = build_graph()
    path = shortest_path(edges, START_NODE, TARGET_NODE)
    waypoints = [node_xy(i, j) for (i, j) in path]

    sx, sy = START_XY
    tx, ty = TARGET_XY
    gx_min, gx_max = min(AVENUES) - 60, max(AVENUES) + 60
    gy_min, gy_max = min(STREETS) - 60, max(STREETS) + 60

    parts = []
    parts.append('<?xml version="1.0" ?>\n<sdf version="1.6">\n')
    parts.append(f'  <world name="city_seed{seed}">\n\n')
    parts.append('    <include><uri>model://sun</uri></include>\n')
    parts.append('    <include><uri>model://ground_plane</uri></include>\n')
    ground_w, ground_len = (gy_max - gy_min) + 100, (gx_max - gx_min) + 100
    ground_cx, ground_cy = (gx_min + gx_max) / 2, (gy_min + gy_max) / 2
    parts.append('    <model name="ground_cover"><static>true</static>'
                  f'<pose>{ground_cx:.1f} {ground_cy:.1f} 0.001 0 0 0</pose><link name="l">'
                  f'<visual name="v"><geometry><box><size>{ground_len:.1f} {ground_w:.1f} 0.01</size></box></geometry>'
                  '<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
                  '<name>Gazebo/Grass</name></script></material></visual></link></model>\n\n')
    parts.append('    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.85 0.95 1</background>'
                  '<shadows>false</shadows></scene>\n')
    parts.append('    <physics type="ode"><max_step_size>0.004</max_step_size>'
                  '<real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate>'
                  '</physics>\n')
    parts.append('    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">'
                  '<ros><namespace>/gazebo</namespace></ros><update_rate>50.0</update_rate></plugin>\n\n')
    parts.append(f'    <gui><camera name="user_camera"><pose>{sx-30:.0f} {sy-30:.0f} 120 0 1.0 0.7</pose>\n'
                  '      <track_visual><name>delivery_drone</name><static>false</static>'
                  '<use_model_frame>true</use_model_frame><xyz>-15 0 10</xyz>'
                  '<inherit_yaw>true</inherit_yaw></track_visual></camera></gui>\n\n')

    # 도로망: 그래프의 모든 에지(끊긴 구간 제외) 렌더링
    parts.append('    <!-- 격자 도로망 -->\n')
    for idx, (n1, adj) in enumerate(edges.items()):
        for (n2, _w) in adj:
            if n1 < n2:  # 양방향 중복 방지
                render_road_edge(parts, n1, n2, f"road_{n1[0]}_{n1[1]}_{n2[0]}_{n2[1]}")
    for node in edges:
        render_intersection(parts, node, args)

    # 블록별 건물
    parts.append('\n    <!-- 블록별 건물 (구역: district_for_block) -->\n')
    bcount = 0
    for i in range(len(AVENUES) - 1):
        for j in range(len(STREETS) - 1):
            blds = gen_block_buildings(rng, i, j, args)
            for b in blds:
                name = f"bld_{bcount:04d}"
                bcount += 1
                if not args.mesh:
                    parts.append(box(name, b["x"], b["y"], b["z"], b["sx"], b["sy"], b["sz"], color=b["color"]))
                elif b["kind"] == "house":
                    parts.append(mesh_house(name, b["x"], b["y"], b["mesh_variant"], target_w=b["sx"]))
                elif b["kind"] == "industrial":
                    parts.append(mesh_industrial(name, b["x"], b["y"], b["mesh_variant"], target_w=b["sx"]))
                elif b["kind"] == "industrial_accent":
                    parts.append(mesh_industrial_accent(name, b["x"], b["y"], b["mesh_variant"], target_h=b["sz"]))
                elif b["kind"] == "tower":
                    parts.append(mesh_tower(name, b["x"], b["y"], b["mesh_variant"], target_h=b["sz"],
                                             collision_w=b["sx"], collision_d=b["sy"]))

    # 맨해튼풍 보조 구역 (다양한 목적지용 — 기존 도시/경로 그래프와 완전히 별개)
    mh_bounds = None
    if args.manhattan:
        _, mh_bounds = build_manhattan_xml(parts, rng, args, bcount)

    # 마을 보조 구역 (비격자, 낮은 집 — 기존 도시/맨해튼과 완전히 별개)
    village_bounds = None
    village_spots = []
    if args.village:
        village_bounds, village_spots = build_village_xml(parts, rng, args)

    # 배송 목적지 후보 (클릭 리타겟용) — 성격 다른 착륙 시나리오 + 이름표 깃발
    spots = landing_spots_static() + village_spots
    parts.append('\n    <!-- 배송 목적지 후보 (착륙 시나리오 + 마커) -->\n')
    for si, s in enumerate(spots):
        # 외곽에 뚝 떨어진 목적지는 가장 가까운 격자 가장자리에서 진입로를 이어 붙인다
        if s["kind"] in ("plaza", "park", "parking", "vacant") and abs(s["x"]) <= 260 and abs(s["y"]) <= 260 \
                and not (min(AVENUES) <= s["x"] <= max(AVENUES) and min(STREETS) <= s["y"] <= max(STREETS)):
            ex = max(min(s["x"], max(AVENUES)), min(AVENUES))
            ey = max(min(s["y"], max(STREETS)), min(STREETS))
            if math.hypot(s["x"] - ex, s["y"] - ey) > 8.0:
                acc = generate_connector_path(rng, (ex, ey), (s["x"], s["y"]), seg_len=12.0)
                render_village_path(parts, acc, f"acc{si}", road_w=6.0)
        render_landing_spot(parts, f"spot{si}", s, rng)

    # 침입 보행자 + 기존 목적지 마커 (mission_controller.py INTRUDER_WAIT_XY와 좌표 일치)
    parts.append('\n    <!-- 침입 보행자 + 배송 목적지 -->\n')
    parts.append(person("person_intruder", INTRUDER_WAIT_XY[0], INTRUDER_WAIT_XY[1],
                         static=False, color=(0.9, 0.55, 0.05)))
    parts.append(f'    <model name="delivery_target_marker"><static>true</static>'
                  f'<pose>{tx:.1f} {ty:.1f} 0.12 0 0 0</pose><link name="l">'
                  f'<visual name="v"><geometry><sphere><radius>0.12</radius></sphere></geometry>'
                  f'<material><ambient>0.1 0.85 0.2 0.8</ambient></material></visual></link></model>\n')

    parts.append('\n')
    parts.append(DRONE_TEMPLATE.format(sx=sx, sy=sy, sz=DRONE_SPAWN_Z))
    parts.append('\n  </world>\n</sdf>\n')
    return "".join(parts), waypoints, path, mh_bounds, village_bounds, spots


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None, help="출력 .world 경로 (기본: generated/city_seed{N}.world)")
    ap.add_argument("--buildings-per-block", type=int, default=6)
    ap.add_argument("--max-height", type=float, default=90.0)
    ap.add_argument("--mesh", action="store_true")
    ap.add_argument("--manhattan", dest="manhattan", action="store_true", default=True,
                     help="기존 도시 옆에 스카이스크래퍼 전용 맨해튼풍 구역 추가 (기본 on)")
    ap.add_argument("--no-manhattan", dest="manhattan", action="store_false",
                     help="맨해튼풍 구역 생략 (빠른 반복 테스트용)")
    ap.add_argument("--village", dest="village", action="store_true", default=True,
                     help="기존 도시 반대편에 비격자(굽이치는 길) 낮은 집 마을 추가 (기본 on)")
    ap.add_argument("--no-village", dest="village", action="store_false",
                     help="마을 구역 생략 (빠른 반복 테스트용)")
    args = ap.parse_args()

    xml_text, waypoints, path, mh_bounds, village_bounds, landing_spots = build_city_xml(args, args.seed)
    validate_xml(xml_text, f"city seed{args.seed}")

    if args.out:
        out_path = args.out
    else:
        out_dir = os.path.join(HERE, "generated")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f"city_seed{args.seed}.world")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(xml_text)

    wp_data = {"start_xy": list(START_XY), "target_xy": list(TARGET_XY),
               "path_nodes": path, "waypoints_xy": [list(w) for w in waypoints],
               "landing_spots": [{"x": s["x"], "y": s["y"], "label": s["label"],
                                   "kind": s["kind"], "note": s["note"]} for s in landing_spots]}
    if mh_bounds is not None:
        gx_min, gx_max, gy_min, gy_max = mh_bounds
        # 맨해튼풍 구역 중심 — 경로계획엔 안 쓰이고, click-to-retarget으로 목적지를 바꿀 때
        # RViz/Gazebo에서 어디를 클릭하면 되는지 참고용으로만 남겨둔다.
        wp_data["manhattan_district_center_xy"] = [round((gx_min + gx_max) / 2, 1),
                                                     round((gy_min + gy_max) / 2, 1)]
        wp_data["manhattan_district_bounds_xy"] = [round(gx_min, 1), round(gx_max, 1),
                                                     round(gy_min, 1), round(gy_max, 1)]
    if village_bounds is not None:
        vx_min, vx_max, vy_min, vy_max = village_bounds
        wp_data["village_district_center_xy"] = [round((vx_min + vx_max) / 2, 1),
                                                   round((vy_min + vy_max) / 2, 1)]
        wp_data["village_district_bounds_xy"] = [round(vx_min, 1), round(vx_max, 1),
                                                   round(vy_min, 1), round(vy_max, 1)]
    wp_path = os.path.splitext(out_path)[0] + ".waypoints.json"
    with open(wp_path, "w", encoding="utf-8") as f:
        json.dump(wp_data, f, ensure_ascii=False, indent=2)

    print(f"[ok] world  -> {out_path} ({len(xml_text)} bytes)")
    print(f"[ok] path   -> {wp_path}")
    print(f"[ok] {len(path)}개 노드, {len(edges_on_path(path))}개 구간, 꺾인 횟수(방향 전환): "
          + str(sum(1 for a, b, c in zip(path, path[1:], path[2:])
                    if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]))))
    print("경로:", " -> ".join(f"({x:.0f},{y:.0f})" for x, y in waypoints))
    if mh_bounds is not None:
        print(f"[ok] 맨해튼풍 구역 중심 -> ({wp_data['manhattan_district_center_xy'][0]}, "
              f"{wp_data['manhattan_district_center_xy'][1]}) — 클릭 리타겟용 참고 좌표")
    if village_bounds is not None:
        print(f"[ok] 마을 구역 중심 -> ({wp_data['village_district_center_xy'][0]}, "
              f"{wp_data['village_district_center_xy'][1]}) — 클릭 리타겟용 참고 좌표")
    print(f"[ok] 배송 목적지 후보 {len(landing_spots)}곳 (클릭 리타겟용):")
    for s in landing_spots:
        print(f"     - ({s['x']:>7.1f}, {s['y']:>7.1f})  {s['label']}  [{s['kind']}]")


if __name__ == "__main__":
    main()
