#!/usr/bin/env python3
"""
generate_world.py — 절차생성 방식 Gazebo Classic .world 생성기 (Fuel 에셋 미사용)

배경 (2026-08-28 설계 확정, ../CONTEXT.md 결정 5):
  기존 residential_delivery.world 는 Fuel 모델(House/Tree/Car/…)을 <include>로
  가져다 썼는데, 2026-06-10 Fuel 메시 로드 실패로 막혔다. 이 스크립트는 그 부분을
  전부 박스/실린더/구 같은 SDF 기본 지오메트리로 대체해 절차생성한다.
    → 넓게 / 집 많게 / 높이 제각각 / 건물 때문에 회피 어려움 (박스 파라미터로 전부 해결)
    → 랜덤 시드로 N개 생성 → 정량실험(성공률) 인프라가 부수효과로 생김 (남은 작업 #4)

바뀌지 않는 것 (mission_controller.py / landing_detector.py 가 이 값에 의존함):
  - START_XY=(-150,0), TARGET_XY=(150,0)   ← mission_controller.py 상수와 반드시 일치
  - 모델명 "delivery_drone"(센서 4개), "person_intruder"(INTRUDER_WAIT_XY=(150,9))
  - 목적지 부근 "채점용" 시나리오는 구조를 유지하고 소재만 박스/실린더로 교체:
      · low_garage — 1.3m 솟은 지붕 (OBSTACLE_DEPTH_MARGIN=0.45 초과 → depth로 거부되는지 테스트)
      · 차량 클러스터 — 도로와 색이 달라 같은 군집으로 안 묶이는지 테스트 (GROUND_COLOR_THRESH)
      · 잔디 — "주 착륙표면과 다른 표면" 배제 테스트
      · person_1/2 — 정적 보행자, person_intruder — 하강 중 개입하는 동적 보행자
  이 시나리오까지 랜덤화하면 그동안 검증한 알고리즘 동작(장애물 회피 이유)이 흔들리므로,
  --seed 는 이 구간에는 "약한 지터"만 주고 구조는 고정한다.

절차생성 대상 (매 seed 다름):
  1) 이착륙 회랑(x: -140~130)의 배경 건물들 — 높이/폭/좌우 오프셋 랜덤, 밀도 --n-buildings
  2) 시케인(슬라럼) 타워 쌍 — 원본의 t_swerve/t_side/t_pair 자리를 대체.
     중앙 통로를 의도적으로 좁혀 전방 depth 회피(compute_avoid_offset)가 반드시 발동하게 함.
     (완전 랜덤 배치만 하면 운 나쁘면 중앙이 뻥 뚫려 회피 테스트 자체가 안 됨 → 구조는 고정,
      치수만 랜덤)
  3) 배경 장식(가로수/가로등) — 순수 시각적 밀도용, 충돌 없음

사용법:
  python3 generate_world.py --seed 3
  python3 generate_world.py --batch 10 --outdir generated   # seed 0..9, manifest.json 생성
  python3 generate_world.py --seed 0 --n-buildings 60 --max-height 110
  python3 generate_world.py --seed 0 --mesh      # 저층주택을 Kenney City Kit Suburban 메시로

--mesh (2026-09-02 Suburban, 2026-09-03 Commercial/Industrial/Roads로 확장): 회랑을
"교외 → 산업지대 → 상업/도심(스카이스크래퍼 협곡) → 목적지 교외"로 이어지는 도시로
만든다(district_for_x 참고) — Kenney City Kit Suburban/Commercial/Industrial/Roads
(전부 CC0)를 assimp로 Collada(.dae) 변환해서 씀. 도로도 flat_patch 단색 띠 대신 실제
도로 타일(차선/갓길 텍스처)을 이어붙인다.
  Kenney 팩 공통 변환 절차 (Suburban에서 시행착오 끝에 확립, 이후 3개 팩은 바로 통과):
  1) OBJ+MTL 원본의 map_Kd 텍스처 경로를 obj/mtl과 같은 폴더로 평탄화 (하위폴더 참조는
     Gazebo가 못 찾아 새까맣게 렌더링됨)
  2) assimp CLI로 .dae 변환 (OBJ 그대로 쓰면 벽 UV가 깨져 지붕만 맞고 벽은 어둡게 나옴)
  3) 변환된 .dae의 <ambient>를 0 0 0 1 → 0.6 0.6 0.6 1로 정규식 치환 (assimp 기본값이 0이라
     태양광이 안 닿는 면은 재질과 무관하게 새까매짐 — 벽 UV 문제로 오인하기 쉬운 함정이었음)
  4) <visual>에 roll=+π/2 보정 (.dae의 up_axis 메타데이터를 Gazebo Classic 11이 실제로는
     안 읽어서, OBJ 때와 똑같이 수동으로 세워야 함)
  충돌은 여전히 원본 절차생성 로직(box 기반 footprint)을 그대로 쓴다 — 시각 메시만 갈아
  끼우는 구조라 eval/gt_mask.py 등 GT 판정 코드는 손 안 대도 된다.
"""
import argparse
import json
import math
import os
import random

# ── mission_controller.py 와 반드시 일치해야 하는 상수 ──
START_XY = (-150.0, 0.0)
TARGET_XY = (150.0, 0.0)
INTRUDER_WAIT_XY = (150.0, 9.0)
DRONE_SPAWN_Z = 7.0

HERE = os.path.dirname(os.path.abspath(__file__))

# ── Kenney City Kit Suburban (CC0) — --mesh 옵션용. bbox는 각 OBJ의 실제 정점 범위를
# 직접 재서 나온 값(폭 w / 높이 h / 깊이 d, OBJ 원본 Y-up 기준). 배치할 때 원하는 폭(target_w)에
# 맞춰 이 native w로 나눈 비율만큼 xyz를 똑같이(uniform) 스케일한다 — 그래야 실제 주택
# 비율이 안 뒤틀림. ──
KENNEY_DIR = os.path.join(HERE, "meshes", "kenney_city_kit_suburban_dae")
MESH_EXT = ".dae"
# 2026-09-02 실측 이력 (worlds/meshes/kenney_city_kit_suburban/ 의 원본 OBJ+MTL 기준):
#   1차: Textures/colormap.png(하위폴더)를 참조해서 Gazebo가 텍스처를 못 찾아 새까맣게
#        렌더링됨 → colormap.png를 obj와 같은 폴더로 평탄화해서 해결(텍스처 로드는 성공,
#        Ogre 로그로 확인함).
#   2차: 텍스처는 로드되는데 지붕만 제대로 된 색이 나오고 벽은 여전히 어둡게 나옴 —
#        Kenney 공식 프리뷰(밝은 벽+창문)와 다름. OBJ의 UV/normal 처리가 Gazebo의
#        Assimp 경로에서 일부만 맞는 것으로 추정.
#   3차(현재): assimp CLI로 OBJ→Collada(.dae) 변환. .dae는 <up_axis>Y_UP</up_axis>
#        메타데이터를 명시적으로 가지고 있고, Gazebo의 Collada 로더는 이걸 보고 자동으로
#        Z-up으로 돌려준다고 알려져 있음(OBJ엔 이런 메타데이터 자체가 없어서 내가 직접
#        <visual> pose에 roll을 넣어야 했던 것과 대조적) — 그래서 mesh_house()/mesh_tree()의
#        수동 roll=+π/2 보정을 뺐다. ⚠️ 이것도 아직 실제로 벽 텍스처가 제대로 나오는지
#        눈으로 확인 못 함 — 벽에 창문/문이 제대로 보이는지 이번에도 반드시 확인 필요.
BUILDING_MESHES = {
    "building-type-a": (1.300, 0.834, 1.028), "building-type-b": (1.828, 1.137, 1.140),
    "building-type-c": (1.286, 1.034, 1.028), "building-type-d": (1.756, 1.238, 1.028),
    "building-type-e": (1.300, 1.137, 1.028), "building-type-f": (1.428, 1.137, 1.406),
    "building-type-g": (1.450, 0.768, 1.178), "building-type-h": (1.300, 0.738, 0.916),
    "building-type-i": (1.286, 0.738, 1.028), "building-type-j": (1.370, 1.038, 0.916),
    "building-type-k": (0.921, 1.150, 1.020), "building-type-l": (1.034, 1.049, 1.020),
    "building-type-m": (1.428, 0.738, 1.428), "building-type-n": (1.784, 1.137, 1.378),
    "building-type-o": (1.270, 1.137, 1.028), "building-type-p": (1.240, 0.918, 0.990),
    "building-type-q": (1.240, 0.918, 0.886), "building-type-r": (1.028, 1.141, 1.020),
    "building-type-s": (1.406, 1.137, 1.086), "building-type-t": (1.314, 1.156, 1.406),
    "building-type-u": (1.428, 1.137, 1.087),
}
TREE_MESHES = {"tree-large": (0.210, 0.767, 0.243), "tree-small": (0.210, 0.567, 0.243)}

# ── 2026-09-03: 도시 확장 — Kenney City Kit Commercial(스카이스크래퍼)/Industrial(공장)/
# Roads(도로 타일). "동네 하나"가 아니라 "교외→산업지대→도심→다시 교외"로 이어지는 도시를
# 만들어달라는 요청(그리고 바닥이 하늘로 뚝 끊기는 문제, 도로가 일자라 그냥 따라가는 것
# 처럼 보이는 문제)에 대응. suburban과 완전히 같은 파이프라인(assimp OBJ→DAE, ambient
# 0→0.6 패치, roll=+π/2)으로 변환했고 처음부터 문제없이 통과함 — 여기 겪었던 3단계 삽질이
# 정말로 "Kenney 팩 공통 특성"이었다는 뜻.
SKY_DIR = os.path.join(HERE, "meshes", "kenney_city_kit_commercial_dae")
SKY_MESHES = {
    "building-skyscraper-a": (1.360, 2.880, 1.360), "building-skyscraper-b": (1.360, 4.480, 1.360),
    "building-skyscraper-c": (1.280, 4.080, 1.388), "building-skyscraper-d": (1.360, 3.680, 1.360),
    "building-skyscraper-e": (1.280, 4.880, 1.280),
}
IND_DIR = os.path.join(HERE, "meshes", "kenney_city_kit_industrial_dae")
IND_MESHES = {
    "building-a": (2.084, 1.470, 1.242), "building-b": (1.428, 1.470, 1.242),
    "building-c": (1.684, 1.650, 1.290), "building-d": (2.084, 1.650, 1.290),
    "building-e": (1.684, 1.650, 1.290), "building-f": (1.428, 1.470, 1.428),
}
IND_ACCENT_MESHES = {"chimney-large": (1.080, 1.700, 1.080), "water-tower": (0.852, 2.142, 0.832)}
ROAD_DIR = os.path.join(HERE, "meshes", "kenney_city_kit_roads_dae")
ROAD_TILE_NATIVE = 1.0  # road-straight.obj 실측: 정확히 1x1m 모듈러 타일, 원점이 중심


# ───────────────────────── SDF 프리미티브 헬퍼 ─────────────────────────

def box(name, x, y, z, sx, sy, sz, yaw=0.0, color=(0.5, 0.5, 0.55), collide=True):
    r, g, b = color
    col = (f'<collision name="c"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}'
           f'</size></box></geometry></collision>') if collide else ""
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.3f}</pose><link name="l">'
            f'{col}'
            f'<visual name="v"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>'
            f'<material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>'
            f'<diffuse>{min(r+0.05,1):.2f} {min(g+0.05,1):.2f} {min(b+0.05,1):.2f} 1</diffuse>'
            f'</material></visual></link></model>\n')


def flat_patch(name, x, y, sx, sy, color, z=0.02):
    """도로/잔디/보도 같은 평평한 바닥 패치 (충돌 없음)."""
    r, g, b = color
    return (f'<model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} {z:.3f} 0 0 0</pose>'
            f'<link name="l"><visual name="v"><geometry><box><size>{sx:.2f} {sy:.2f} 0.02</size></box>'
            f'</geometry><material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient></material></visual>'
            f'</link></model>\n')


def tree(name, x, y, trunk_h, canopy_r, green):
    """트렁크(실린더) + 수관(구) — Fuel Pine/Oak Tree 대체. 치수는 generate_layout()에서 미리 뽑아둔 값."""
    return (f'<model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose>'
            f'<link name="l">'
            f'<visual name="trunk"><pose>0 0 {trunk_h/2:.2f} 0 0 0</pose>'
            f'<geometry><cylinder><radius>0.12</radius><length>{trunk_h:.2f}</length></cylinder></geometry>'
            f'<material><ambient>0.35 0.25 0.15 1</ambient></material></visual>'
            f'<visual name="canopy"><pose>0 0 {trunk_h + canopy_r*0.7:.2f} 0 0 0</pose>'
            f'<geometry><sphere><radius>{canopy_r:.2f}</radius></sphere></geometry>'
            f'<material><ambient>0.15 {green:.2f} 0.18 1</ambient></material></visual>'
            f'</link></model>\n')


def lamp_post(name, x, y):
    return (f'<model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose>'
            f'<link name="l">'
            f'<visual name="pole"><pose>0 0 1.6 0 0 0</pose>'
            f'<geometry><cylinder><radius>0.05</radius><length>3.2</length></cylinder></geometry>'
            f'<material><ambient>0.15 0.15 0.17 1</ambient></material></visual>'
            f'<visual name="lamp"><pose>0 0 3.25 0 0 0</pose>'
            f'<geometry><sphere><radius>0.14</radius></sphere></geometry>'
            f'<material><ambient>0.95 0.9 0.6 1</ambient></material></visual>'
            f'</link></model>\n')


def mesh_house(name, x, y, model_name, target_w, yaw=0.0, color=None):
    """Kenney 건물 메시(.dae). 충돌은 여전히 단순 박스(collide 논리·GT마스크 그대로 재사용).
    2026-09-02 실측: Gazebo Classic 11은 .dae의 up_axis 메타데이터를 실제로는 안 읽는다
    (그래서 이걸 믿고 회전 보정을 뺐다가 건물이 다시 누움 — 사용자가 스크린샷으로 확인).
    OBJ 때 검증됐던 것과 같은 roll=+π/2 보정을 <visual>에 다시 넣는다 — assimp가 OBJ→DAE로
    변환할 때 정점 좌표값 자체는 그대로 옮기고 up_axis 태그만 정보성으로 붙이는 것으로 보임.
    color (2026-09-04, "집 색상 바꿀 수 없냐" 질문에 대응): SDF <visual>에 <material>을
    얹으면 실측(색상 테스트 월드, down_cam 캡처로 확인)상 Gazebo Classic 11이 메시에 박힌
    원본 텍스처(벽 무늬)를 완전히 무시하고 단색으로 덮어써버린다 — 창문/문 같은 지오메트리
    굴곡은 음영으로 여전히 보이지만 벽 색 텍스처 디테일은 사라진다. 그 트레이드오프를
    감수하고 마을 집처럼 "집집마다 다른 색"을 내고 싶을 때만 color를 넘기고, 기존 호출부
    (교외/도심 등)는 None을 유지해 원본 텍스처를 그대로 쓴다."""
    nw, nh, nd = BUILDING_MESHES[model_name]
    scale = target_w / nw
    h, d = nh * scale, nd * scale
    uri = f"file://{KENNEY_DIR}/{model_name}{MESH_EXT}"
    mat = ""
    if color is not None:
        r, g, b = color
        mat = f'<material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse></material>'
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.3f}</pose><link name="l">'
            f'<collision name="c"><pose>0 0 {h/2:.3f} 0 0 0</pose>'
            f'<geometry><box><size>{target_w:.2f} {d:.2f} {h:.2f}</size></box></geometry></collision>'
            f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
            f'<geometry><mesh><uri>{uri}</uri>'
            f'<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale></mesh></geometry>{mat}</visual>'
            f'</link></model>\n')


def mesh_tree(name, x, y, model_name, target_h):
    """Kenney 나무 메시(.dae) (충돌 없음 — 기존 tree() 프리미티브와 동일하게 장식용).
    mesh_house()와 동일한 이유로 roll=+π/2 보정 필요."""
    nw, nh, nd = TREE_MESHES[model_name]
    scale = target_h / nh
    uri = f"file://{KENNEY_DIR}/{model_name}{MESH_EXT}"
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} 0 0 0 0</pose><link name="l">'
            f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
            f'<geometry><mesh><uri>{uri}</uri>'
            f'<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale></mesh></geometry></visual>'
            f'</link></model>\n')


def mesh_industrial(name, x, y, model_name, target_w, yaw=0.0):
    """산업지구 건물(공장/창고) 메시. mesh_house()와 동일한 규칙(폭 기준 스케일,
    roll=+π/2, 충돌은 별도 박스)."""
    nw, nh, nd = IND_MESHES[model_name]
    scale = target_w / nw
    h, d = nh * scale, nd * scale
    uri = f"file://{IND_DIR}/{model_name}{MESH_EXT}"
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.3f}</pose><link name="l">'
            f'<collision name="c"><pose>0 0 {h/2:.3f} 0 0 0</pose>'
            f'<geometry><box><size>{target_w:.2f} {d:.2f} {h:.2f}</size></box></geometry></collision>'
            f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
            f'<geometry><mesh><uri>{uri}</uri>'
            f'<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale></mesh></geometry></visual>'
            f'</link></model>\n')


def mesh_industrial_accent(name, x, y, model_name, target_h):
    """굴뚝/급수탑 — 산업지구 높이 포인트 장식 (충돌 없음, 순수 스카이라인용)."""
    nw, nh, nd = IND_ACCENT_MESHES[model_name]
    scale = target_h / nh
    uri = f"file://{IND_DIR}/{model_name}{MESH_EXT}"
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} 0 0 0 0</pose><link name="l">'
            f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
            f'<geometry><mesh><uri>{uri}</uri>'
            f'<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale></mesh></geometry></visual>'
            f'</link></model>\n')


def mesh_tower(name, x, y, model_name, target_h, collision_w, collision_d, yaw=0.0):
    """상업지구 스카이스크래퍼(협곡/시케인용). 시각 메시는 높이(target_h) 기준으로 스케일하고,
    충돌 박스는 회피 로직이 요구하는 collision_w/d를 그대로 쓴다 — 시각 폭과 어긋날 수 있지만
    회피 동작(전방 depth가 보는 폭)이 더 중요해서 그쪽을 우선했다."""
    nw, nh, nd = SKY_MESHES[model_name]
    scale = target_h / nh
    uri = f"file://{SKY_DIR}/{model_name}{MESH_EXT}"
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.3f}</pose><link name="l">'
            f'<collision name="c"><pose>0 0 {target_h/2:.3f} 0 0 0</pose>'
            f'<geometry><box><size>{collision_w:.2f} {collision_d:.2f} {target_h:.2f}</size></box></geometry></collision>'
            f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
            f'<geometry><mesh><uri>{uri}</uri>'
            f'<scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale></mesh></geometry></visual>'
            f'</link></model>\n')


def road_tile(name, x, y, cross_w, along_len=None, yaw=0.0, variant="straight"):
    """도로 타일 (road-straight/road-bend). 실측(2026-09-03): road-straight.dae 원본은
    X=1m, Z=1m인 정사각 모듈러 타일(Y=0~0.02, 두께 — up_axis=Y_UP 기준). 예전엔 <scale>을
    등방(가로=세로=tile_w)으로만 줘서 타일 하나가 항상 정사각형(예: 14x14)이었고, 구간
    길이가 tile_w의 배수가 아니면 나머지만큼 도로 사이사이가 비어 보였다("도로가 다 조금씩
    띄어져있네" — 2026-09-03 사용자 스크린샷). along_len을 cross_w와 별도로 줘서 진행방향만
    늘이거나 줄이면 폭(교차로와 맞물리는 실제 도로 폭)은 그대로 두고 구간을 빈틈없이 채울
    수 있다.
    ⚠️ 축 대응관계는 직접 렌더링해서 눈으로 확인해야 했음: <visual> pose의 roll=+π/2는
    로컬 X축 기준 회전이라 X는 그대로 X에 남고 Y(원본 up)↔Z가 뒤바뀐다. 그래서 순진하게
    "진행방향=원본 depth축(Z)"라고 가정했다가 첫 시도에서 실제로는 반대(yaw=0일 때 모델
    좌표 X가 그대로 월드 X=스트리트 진행방향이 됨, Z는 롤 이후 월드 Y가 됨)라는 게 축
    테스트 월드(마커 박스로 +X/+Y 표시, down_cam으로 확인)에서 드러났다. 그래서 scale의
    X 성분에 along_len(진행방향), Z 성분에 cross_w(도로 폭)를 넣는다.
    along_len 생략 시(기존 호출부 하위호환) 등방 스케일로 동작. 충돌 없음(도로는 시각용
    — 실제 착륙 안전판정은 landing_detector.py가 depth+색상으로 함)."""
    if along_len is None:
        along_len = cross_w
    fname = "road-straight" if variant == "straight" else "road-bend"
    uri = f"file://{ROAD_DIR}/{fname}{MESH_EXT}"
    scale_cross = cross_w / ROAD_TILE_NATIVE
    scale_along = along_len / ROAD_TILE_NATIVE
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.3f}</pose><link name="l">'
            f'<visual name="v"><pose>0 0 0 1.5708 0 0</pose>'
            f'<geometry><mesh><uri>{uri}</uri>'
            f'<scale>{scale_along:.4f} {scale_cross:.4f} {scale_cross:.4f}</scale></mesh></geometry></visual>'
            f'</link></model>\n')


def car(name, x, y, yaw, color):
    """Fuel Hatchback/Pickup 대체 — 도로면과 확실히 다른 색으로 '차 클러스터' 배제 테스트 유지."""
    return box(name, x, y, 0.55, 4.2, 1.8, 1.1, yaw=yaw, color=color)


def cone(name, x, y):
    return (f'<model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose>'
            f'<link name="l"><visual name="v"><pose>0 0 0.25 0 0 0</pose>'
            f'<geometry><cylinder><radius>0.18</radius><length>0.5</length></cylinder></geometry>'
            f'<material><ambient>0.95 0.4 0.05 1</ambient></material></visual></link></model>\n')


def person(name, x, y, static=True, color=(0.5, 0.5, 0.5)):
    r, g, b = color
    kin = "" if static else "<gravity>false</gravity><kinematic>true</kinematic>" \
        "<inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.05</izz>" \
        "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
    return (f'<model name="{name}"><static>{"true" if static else "false"}</static>'
            f'<pose>{x:.2f} {y:.2f} 0.85 0 0 0</pose><link name="l">{kin}'
            f'<collision name="c"><geometry><cylinder><radius>0.25</radius><length>1.7</length>'
            f'</cylinder></geometry></collision>'
            f'<visual name="b"><geometry><cylinder><radius>0.25</radius><length>1.4</length></cylinder>'
            f'</geometry><material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient></material></visual>'
            f'<visual name="h"><pose>0 0 0.9 0 0 0</pose><geometry><sphere><radius>0.16</radius></sphere>'
            f'</geometry><material><ambient>0.8 0.65 0.5 1</ambient></material></visual>'
            f'</link></model>\n')


# ───────────────────────── 절차생성 로직 ─────────────────────────

def overlaps(x, y, hx, hy, placed, margin=1.5):
    for (px, py, phx, phy) in placed:
        if abs(x - px) < (hx + phx + margin) and abs(y - py) < (hy + phy + margin):
            return True
    return False


def district_for_x(x, x_min, x_max):
    """회랑 위치(x)로 도시 구역을 결정 — 교외(출발) → 산업지대 → 상업/도심(협곡) → 교외(도착
    직전, 목적지 존 자체가 이미 교외 스타일이라 자연스럽게 이어짐). 남은작업 #5 논의:
    "마을/도시가 다 이어져 있는 큰 지도"로 확장."""
    frac = (x - x_min) / max(x_max - x_min, 1e-6)
    if frac < 0.3:
        return "suburban"
    if frac < 0.6:
        return "industrial"
    return "commercial"


def gen_corridor_buildings(rng, x_min, x_max, n, lane_offset, half_width, h_min, h_max):
    """회랑 배경 건물. 좌우 차선 바깥쪽에 랜덤 배치 — '넓게/집 많게/높이 제각각'.
    구역(district_for_x)에 따라 종류·메시 풀이 바뀐다: 교외=주택, 산업지대=공장(+굴뚝/급수탑
    억양), 상업=스카이스크래퍼 위주(+낮은 상가 소수)."""
    out, placed = [], []
    tries = 0
    while len(out) < n and tries < n * 30:
        tries += 1
        x = rng.uniform(x_min, x_max)
        side = rng.choice((-1, 1))
        y = side * rng.uniform(lane_offset, half_width)
        sx = rng.uniform(5, 12)
        sy = rng.uniform(5, 12)
        district = district_for_x(x, x_min, x_max)

        if district == "suburban":
            h, kind, mesh_variant = rng.uniform(h_min, 10), "house", rng.choice(list(BUILDING_MESHES))
        elif district == "industrial":
            if rng.random() < 0.85:
                h, kind = rng.uniform(6, 14), "industrial"
                mesh_variant = rng.choice(list(IND_MESHES))
            else:
                h, kind = rng.uniform(18, min(28, h_max)), "industrial_accent"
                mesh_variant = rng.choice(list(IND_ACCENT_MESHES))
                sx = sy = rng.uniform(1.5, 3.0)  # 굴뚝/급수탑은 가늘어서 GT 판정용 footprint도 줄임
        else:  # commercial
            if rng.random() < 0.75:
                h, kind, mesh_variant = rng.uniform(25, h_max), "tower", rng.choice(list(SKY_MESHES))
            else:
                h, kind, mesh_variant = rng.uniform(h_min, 10), "house", rng.choice(list(BUILDING_MESHES))

        if overlaps(x, y, sx / 2, sy / 2, placed):
            continue
        placed.append((x, y, sx / 2, sy / 2))
        hue = rng.uniform(0.35, 0.65)
        out.append({"x": x, "y": y, "z": h / 2, "sx": sx, "sy": sy, "sz": h, "kind": kind,
                     "district": district, "mesh_variant": mesh_variant,
                     "color": (hue, hue - 0.05, hue - 0.1)})
    return out


def gen_chicane(rng, x_min, x_max, n_gates, base_half_gap, h_min, h_max):
    """중앙 통로를 좁히는 슬라럼 타워 쌍 — 원본 t_swerve/t_side/t_pair를 일반화.
    좌우 안전폭을 번갈아 좁혀서 전방 depth 회피가 반드시 발동하도록 구조를 고정하고,
    치수(높이/간격/좌우 어느 쪽이 좁은지)만 시드로 랜덤화한다."""
    gates = []
    xs = [x_min + (i + 1) * (x_max - x_min) / (n_gates + 1) for i in range(n_gates)]
    bias = rng.choice((-1, 1))  # 첫 게이트가 어느 쪽으로 치우칠지
    for i, gx in enumerate(xs):
        bias *= -1  # 좌우 번갈아 → 슬라럼
        gap = base_half_gap * rng.uniform(0.55, 0.85)  # AVOID_RANGE_M(22m) 안에서 반드시 걸리도록 좁게
        offset = bias * rng.uniform(2.0, 5.0)          # 어느 쪽 타워가 더 안쪽으로 들어오는지
        h_l = rng.uniform(h_min, h_max)
        h_r = rng.uniform(h_min, h_max)
        w = rng.uniform(7, 11)
        # --mesh 여부와 무관하게 항상 뽑아둠 (다른 mesh_variant들과 같은 재현성 원칙)
        mv_l, mv_r = rng.choice(list(SKY_MESHES)), rng.choice(list(SKY_MESHES))
        gates.append({
            "x": gx,
            "l": {"y": gap + max(0, offset), "h": h_l, "w": w, "mesh_variant": mv_l},
            "r": {"y": -gap + min(0, offset), "h": h_r, "w": w, "mesh_variant": mv_r},
        })
    return gates


def gen_decorations(rng, x_min, x_max, lane_offset, n):
    """회랑 배경 장식 (가로수/가로등) — 충돌 없는 순수 밀도용. eval/gt_mask.py는 이걸 장애물로 안 침."""
    out = []
    for i in range(n):
        x = rng.uniform(x_min, x_max)
        y = rng.choice((-1, 1)) * rng.uniform(lane_offset - 3, lane_offset + 1)
        if rng.random() < 0.6:
            out.append({"kind": "tree", "x": x, "y": y, "trunk_h": rng.uniform(1.6, 2.4),
                        "canopy_r": rng.uniform(1.0, 1.8), "green": rng.uniform(0.35, 0.55),
                        "mesh_variant": rng.choice(list(TREE_MESHES))})
        else:
            out.append({"kind": "lamp", "x": x, "y": y})
    return out


def gen_target_zone(rng, jitter):
    """목적지 부근 '채점용' 시나리오. 구조 고정, 위치만 ±jitter(m) 흔든다."""
    def j():
        return rng.uniform(-jitter, jitter)
    tx, ty = TARGET_XY
    tree_xy = [(tx + 9 + j(), ty + 8 + j()), (tx - 8 + j(), ty - 2 + j())]
    trees = [{"x": x, "y": y, "trunk_h": rng.uniform(1.6, 2.4),
              "canopy_r": rng.uniform(1.0, 1.8), "green": rng.uniform(0.35, 0.55),
              "mesh_variant": rng.choice(list(TREE_MESHES))}
             for (x, y) in tree_xy]
    house_xy = [(tx - 2, ty + 17), (tx - 2, ty - 17), (tx + 25, ty + 12), (tx + 25, ty - 12)]
    houses = [{"x": x, "y": y, "mesh_variant": rng.choice(list(BUILDING_MESHES))}
              for (x, y) in house_xy]
    return {
        "garage": (tx - 6 + j(), ty - 6 + j()),
        "cars": [
            (tx + 4 + j(), ty + 6.5 + j(), 0.3),
            (tx + 6 + j(), ty + 2.5 + j(), 1.55),
            (tx - 3 + j(), ty + 7.5 + j(), 0.1),
            (tx - 7 + j(), ty - 10 + j(), -0.2),
        ],
        "grass": (tx + 12 + j(), ty - 1 + j()),
        "cones": [(tx + 6.5 + j(), ty + 8.5 + j()), (tx + 8 + j(), ty + 4.0 + j())],
        "trees": trees,
        "lamps": [(tx + 3 + j(), ty - 7.5 + j()), (tx - 3 + j(), ty + 6 + j())],
        "houses": houses,
        "persons": [(tx + 2, ty + 8), (tx - 3, ty - 8.5)],
    }


# 데이터-only 절차생성 — build_world_xml()과 eval/gt_mask.py 양쪽이 공유하는 단일 출처.
# 순서(건물 → 장식 → 시케인 → 목적지존)는 build_world_xml() 예전 로직과 동일하게 유지해야
# 같은 seed가 이전과 동일한 배치를 내놓는다.
def generate_layout(args, seed):
    rng = random.Random(seed)
    sx, sy = START_XY
    tx, ty = TARGET_XY
    corridor_x_min, corridor_x_max = sx + 15, tx - 25
    buildings = gen_corridor_buildings(
        rng, x_min=corridor_x_min, x_max=corridor_x_max, n=args.n_buildings,
        lane_offset=args.lane_offset, half_width=args.half_width,
        h_min=4.0, h_max=args.max_height)
    decos = gen_decorations(rng, x_min=corridor_x_min, x_max=corridor_x_max,
                             lane_offset=args.lane_offset, n=args.n_buildings // 3)
    # 시케인은 상업/도심 구역(district_for_x의 마지막 40%)에 겹치게 배치 — 스카이스크래퍼
    # 협곡이 실제로 "도심 한복판"에서 벌어지는 것처럼 보이게
    commercial_x_min = corridor_x_min + 0.6 * (corridor_x_max - corridor_x_min)
    gates = gen_chicane(rng, x_min=commercial_x_min, x_max=tx - 40, n_gates=args.n_gates,
                         base_half_gap=args.half_width * 0.5, h_min=25, h_max=args.max_height)
    target_zone = gen_target_zone(rng, jitter=args.target_jitter)
    return {"seed": seed, "start_xy": START_XY, "target_xy": TARGET_XY,
            "buildings": buildings, "decos": decos, "gates": gates, "target_zone": target_zone}


DEFAULT_ARGS = dict(n_buildings=40, n_gates=4, lane_offset=11.0, half_width=40.0,
                     max_height=90.0, target_jitter=0.0, mesh=False)


def default_args(**overrides):
    """CLI 없이 generate_layout()/build_world_xml()을 쓰는 코드(eval/*)용 — CLI 기본값과 동일."""
    d = dict(DEFAULT_ARGS)
    d.update(overrides)
    return argparse.Namespace(**d)


# ───────────────────────── 조립 ─────────────────────────

DRONE_TEMPLATE = """    <!-- ════════════════════════════════════════════════════════ -->
    <!-- 배송 드론 — 하강 RGB + 하강 Depth + Chase + 전방 Depth 4센서 (생성기 산출물에서도 불변) -->
    <!-- ════════════════════════════════════════════════════════ -->
    <model name="delivery_drone">
      <pose>{sx} {sy} {sz} 0 0 0</pose>
      <link name="base_link">
        <gravity>false</gravity><kinematic>true</kinematic>
        <inertial><mass>1.6</mass><inertia><ixx>0.03</ixx><iyy>0.03</iyy><izz>0.05</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>

        <visual name="body"><geometry><box><size>0.34 0.34 0.09</size></box></geometry><material><ambient>0.13 0.13 0.16 1</ambient><diffuse>0.18 0.18 0.21 1</diffuse></material></visual>
        <visual name="canopy"><pose>0 0 0.06 0 0 0</pose><geometry><box><size>0.20 0.20 0.05</size></box></geometry><material><ambient>0.85 0.45 0.1 1</ambient><diffuse>0.95 0.5 0.12 1</diffuse></material></visual>
        <visual name="arm_a"><pose>0 0 0 0 0 0.785</pose><geometry><box><size>0.60 0.035 0.022</size></box></geometry><material><ambient>0.08 0.08 0.08 1</ambient></material></visual>
        <visual name="arm_b"><pose>0 0 0 0 0 -0.785</pose><geometry><box><size>0.60 0.035 0.022</size></box></geometry><material><ambient>0.08 0.08 0.08 1</ambient></material></visual>
        <visual name="motor_fl"><pose> 0.21  0.21 0.03 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>0.05</length></cylinder></geometry><material><ambient>0.2 0.2 0.22 1</ambient></material></visual>
        <visual name="motor_fr"><pose> 0.21 -0.21 0.03 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>0.05</length></cylinder></geometry><material><ambient>0.2 0.2 0.22 1</ambient></material></visual>
        <visual name="motor_bl"><pose>-0.21  0.21 0.03 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>0.05</length></cylinder></geometry><material><ambient>0.2 0.2 0.22 1</ambient></material></visual>
        <visual name="motor_br"><pose>-0.21 -0.21 0.03 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>0.05</length></cylinder></geometry><material><ambient>0.2 0.2 0.22 1</ambient></material></visual>
        <visual name="prop_fl"><pose> 0.21  0.21 0.07 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>0.008</length></cylinder></geometry><material><ambient>0.4 0.4 0.45 0.55</ambient><diffuse>0.4 0.4 0.45 0.55</diffuse></material></visual>
        <visual name="prop_fr"><pose> 0.21 -0.21 0.07 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>0.008</length></cylinder></geometry><material><ambient>0.4 0.4 0.45 0.55</ambient><diffuse>0.4 0.4 0.45 0.55</diffuse></material></visual>
        <visual name="prop_bl"><pose>-0.21  0.21 0.07 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>0.008</length></cylinder></geometry><material><ambient>0.4 0.4 0.45 0.55</ambient><diffuse>0.4 0.4 0.45 0.55</diffuse></material></visual>
        <visual name="prop_br"><pose>-0.21 -0.21 0.07 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>0.008</length></cylinder></geometry><material><ambient>0.4 0.4 0.45 0.55</ambient><diffuse>0.4 0.4 0.45 0.55</diffuse></material></visual>
        <visual name="leg_fl"><pose> 0.12  0.12 -0.11 0 0 0</pose><geometry><box><size>0.02 0.02 0.16</size></box></geometry><material><ambient>0.1 0.1 0.1 1</ambient></material></visual>
        <visual name="leg_fr"><pose> 0.12 -0.12 -0.11 0 0 0</pose><geometry><box><size>0.02 0.02 0.16</size></box></geometry><material><ambient>0.1 0.1 0.1 1</ambient></material></visual>
        <visual name="leg_bl"><pose>-0.12  0.12 -0.11 0 0 0</pose><geometry><box><size>0.02 0.02 0.16</size></box></geometry><material><ambient>0.1 0.1 0.1 1</ambient></material></visual>
        <visual name="leg_br"><pose>-0.12 -0.12 -0.11 0 0 0</pose><geometry><box><size>0.02 0.02 0.16</size></box></geometry><material><ambient>0.1 0.1 0.1 1</ambient></material></visual>
        <visual name="foot_f"><pose> 0.12 0 -0.19 0 0 0</pose><geometry><box><size>0.06 0.30 0.02</size></box></geometry><material><ambient>0.1 0.1 0.1 1</ambient></material></visual>
        <visual name="foot_b"><pose>-0.12 0 -0.19 0 0 0</pose><geometry><box><size>0.06 0.30 0.02</size></box></geometry><material><ambient>0.1 0.1 0.1 1</ambient></material></visual>
        <visual name="gimbal"><pose>0 0 -0.085 0 0 0</pose><geometry><sphere><radius>0.045</radius></sphere></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="parcel"><pose>-0.13 0 -0.05 0 0 0</pose><geometry><box><size>0.13 0.13 0.11</size></box></geometry><material><ambient>0.62 0.46 0.3 1</ambient><diffuse>0.7 0.52 0.34 1</diffuse></material></visual>

        <sensor name="down_cam" type="camera"><pose>0.02 0 -0.12 0 1.5708 0</pose><update_rate>15</update_rate>
          <camera><horizontal_fov>1.396</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.1</near><far>400</far></clip></camera>
          <plugin name="down_cam_plugin" filename="libgazebo_ros_camera.so"><ros><namespace>/drone</namespace></ros><camera_name>down_cam</camera_name><frame_name>down_cam_link</frame_name></plugin></sensor>
        <sensor name="down_depth" type="depth"><pose>0.02 0 -0.12 0 1.5708 0</pose><update_rate>10</update_rate>
          <camera><horizontal_fov>1.396</horizontal_fov><image><width>320</width><height>240</height><format>R8G8B8</format></image><clip><near>0.1</near><far>400</far></clip></camera>
          <plugin name="down_depth_plugin" filename="libgazebo_ros_camera.so"><ros><namespace>/drone</namespace></ros><camera_name>down_depth</camera_name><frame_name>down_depth_link</frame_name><min_depth>0.1</min_depth><max_depth>400.0</max_depth></plugin></sensor>
        <sensor name="chase_cam" type="camera"><pose>-2.0 0 3.5 0 0.62 0</pose><update_rate>15</update_rate>
          <camera><horizontal_fov>1.5</horizontal_fov><image><width>640</width><height>400</height><format>R8G8B8</format></image><clip><near>0.05</near><far>500</far></clip></camera>
          <plugin name="chase_cam_plugin" filename="libgazebo_ros_camera.so"><ros><namespace>/drone</namespace></ros><camera_name>chase_cam</camera_name><frame_name>chase_cam_link</frame_name></plugin></sensor>
        <!-- 전방 Depth 카메라 (반응형 회피용) -->
        <sensor name="front_depth" type="depth"><pose>0.2 0 0 0 0 0</pose><update_rate>10</update_rate>
          <camera><horizontal_fov>1.5</horizontal_fov><image><width>320</width><height>240</height><format>R8G8B8</format></image><clip><near>0.3</near><far>200</far></clip></camera>
          <plugin name="front_depth_plugin" filename="libgazebo_ros_camera.so"><ros><namespace>/drone</namespace></ros><camera_name>front_depth</camera_name><frame_name>front_depth_link</frame_name><min_depth>0.3</min_depth><max_depth>200.0</max_depth></plugin></sensor>
        <!-- 아이소메트릭 카메라 (뒤-우 위쪽에서 드론+장면을 대각선으로 — 게임 3인칭 뷰) -->
        <sensor name="iso_cam" type="camera"><pose>-4 3 6 0 0.7 -0.45</pose><update_rate>15</update_rate>
          <camera><horizontal_fov>1.4</horizontal_fov><image><width>640</width><height>400</height><format>R8G8B8</format></image><clip><near>0.05</near><far>500</far></clip></camera>
          <plugin name="iso_cam_plugin" filename="libgazebo_ros_camera.so"><ros><namespace>/drone</namespace></ros><camera_name>iso_cam</camera_name><frame_name>iso_cam_link</frame_name></plugin></sensor>
      </link>
      <plugin name="drone_p3d" filename="libgazebo_ros_p3d.so">
        <ros><namespace>/drone</namespace><remapping>odom:=odom</remapping></ros><body_name>base_link</body_name><frame_name>world</frame_name><update_rate>50</update_rate></plugin>
    </model>
"""


def build_world_xml(args, seed):
    layout = generate_layout(args, seed)
    tx, ty = layout["target_xy"]
    sx, sy = layout["start_xy"]
    road_cx = (sx + tx) / 2.0
    road_len = (tx - sx) + 40.0

    parts = []
    parts.append('<?xml version="1.0" ?>\n<sdf version="1.6">\n')
    parts.append(f'  <world name="residential_delivery_seed{seed}">\n\n')
    parts.append('    <include><uri>model://sun</uri></include>\n')
    parts.append('    <include><uri>model://ground_plane</uri></include>\n')
    # 기본 ground_plane은 몇십m 정도로 작아서, 300m+ 회랑 끝까지 가면 바닥이 뚝 끊기고
    # 하늘 배경이 그대로 보여 "낭떨어지"처럼 보였다(2026-09-03, 사용자 스크린샷으로 확인).
    # ground_plane은 충돌(물리)용으로 그대로 두고, 그 위(z를 살짝 더 크게)에 회랑 전체+여유폭을
    # 덮는 큰 잔디 패치를 시각적으로만 얹어서 가린다 — road(z=0.005)보다는 낮게 깔아야
    # 도로가 잔디 위로 정상적으로 보인다. flat_patch류는 충돌 없음(물리는 안 건드림).
    ground_w = 2.0 * (args.half_width + 60.0)
    ground_len = road_len + 150.0
    parts.append('    <model name="ground_cover"><static>true</static>'
                  f'<pose>{road_cx:.1f} 0 0.001 0 0 0</pose><link name="l">'
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
    parts.append(f'    <gui><camera name="user_camera"><pose>{sx-10:.0f} 0 18 0 0.45 0</pose>\n'
                  '      <track_visual><name>delivery_drone</name><static>false</static>'
                  '<use_model_frame>true</use_model_frame><xyz>-10 0 5</xyz>'
                  '<inherit_yaw>true</inherit_yaw></track_visual></camera></gui>\n\n')

    # 도로 + 중앙선 (절차생성 폭에 맞춰 자동으로 늘어남 → "넓게")
    # 참고: 이 도로는 드론 비행축 "시각화"일 뿐 — mission_controller.py는 차선을 전혀 안 보고
    # 목표점+전방 depth 회피로만 난다(차선인식 없음). 시케인에서 실제로 중앙선 벗어나 최대
    # 9m 스웨브하는 걸로 이미 검증됨 — "그냥 도로 따라가는 거 아니냐"는 그 장면으로 반박됨.
    parts.append(f'    <!-- 전 구간 도로 (드론 비행축 시각화, 알고리즘은 안 씀) -->\n')
    if args.mesh:
        tile_w = 10.0  # road-straight 타일 자체에 차선+갓길이 텍스처로 들어있음
        n_tiles = int(road_len // tile_w) + 1
        start_x = road_cx - (n_tiles * tile_w) / 2.0
        for i in range(n_tiles):
            parts.append(road_tile(f"road_{i:03d}", start_x + i * tile_w + tile_w / 2.0, 0, tile_w))
    else:
        parts.append(flat_patch("road", road_cx, 0, road_len, 10, (0.28, 0.28, 0.30), z=0.005))
        parts.append(flat_patch("centerline", road_cx, 0, road_len, 0.18, (0.9, 0.85, 0.3), z=0.02))

    # 1) 회랑 배경 건물 — 절차생성, seed마다 다름 (교외→산업지대→상업/도심 순서로 구역 전환)
    parts.append('\n    <!-- ① 절차생성 회랑 건물 (seed마다 다름, district_for_x로 구역화) -->\n')
    for i, b in enumerate(layout["buildings"]):
        name = f"bld_{i:03d}"
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
        else:
            parts.append(box(name, b["x"], b["y"], b["z"], b["sx"], b["sy"], b["sz"], color=b["color"]))

    # 배경 장식 (충돌 없는 가로수/가로등 — 순수 밀도용)
    for i, d in enumerate(layout["decos"]):
        if d["kind"] == "tree":
            if args.mesh:
                parts.append(mesh_tree(f"deco_tree_{i:03d}", d["x"], d["y"],
                                        d["mesh_variant"], target_h=d["trunk_h"] + d["canopy_r"] * 1.4))
            else:
                parts.append(tree(f"deco_tree_{i:03d}", d["x"], d["y"],
                                   d["trunk_h"], d["canopy_r"], d["green"]))
        else:
            parts.append(lamp_post(f"deco_lamp_{i:03d}", d["x"], d["y"]))

    # 2) 시케인(슬라럼) 게이트 — 회피가 반드시 발동하도록 구조 고정, 치수만 랜덤.
    # --mesh면 스카이스크래퍼로(상업/도심 구역과 겹치게 배치됨, generate_layout 참고)
    parts.append('\n    <!-- ② 시케인 게이트: 전방 depth 회피(AVOID_RANGE_M) 강제 발동 -->\n')
    for i, g in enumerate(layout["gates"]):
        L, R = g["l"], g["r"]
        if args.mesh:
            parts.append(mesh_tower(f"gate_{i}_l", g["x"], L["y"], L["mesh_variant"],
                                     target_h=L["h"], collision_w=L["w"], collision_d=L["w"]))
            parts.append(mesh_tower(f"gate_{i}_r", g["x"], R["y"], R["mesh_variant"],
                                     target_h=R["h"], collision_w=R["w"], collision_d=R["w"]))
        else:
            parts.append(box(f"gate_{i}_l", g["x"], L["y"], L["h"] / 2, L["w"], L["w"], L["h"],
                              color=(0.45, 0.47, 0.53)))
            parts.append(box(f"gate_{i}_r", g["x"], R["y"], R["h"] / 2, R["w"], R["w"], R["h"],
                              color=(0.5, 0.52, 0.58)))

    # 3) 목적지 부근 '채점용' 시나리오 — 구조 고정, 위치만 소지터
    parts.append('\n    <!-- ③ 목적지 부근 채점용 시나리오 (구조 고정, 위치만 ±jitter) -->\n')
    tz = layout["target_zone"]
    parts.append(flat_patch("asphalt_zone", tx, ty, 34, 28, (0.32, 0.32, 0.34), z=0.01))
    parts.append(flat_patch("lane_a", tx, ty + 3.0, 14, 0.14, (0.9, 0.9, 0.9), z=0.03))
    parts.append(flat_patch("lane_b", tx, ty - 3.0, 14, 0.14, (0.9, 0.9, 0.9), z=0.03))
    for i, hs in enumerate(tz["houses"]):
        if args.mesh:
            parts.append(mesh_house(f"house_{i}", hs["x"], hs["y"], hs["mesh_variant"], target_w=9.0))
        else:
            parts.append(box(f"house_{i}", hs["x"], hs["y"], 3.0, 9, 8, 6.0, color=(0.6, 0.55, 0.45)))
    gx, gy = tz["garage"]
    parts.append(box("low_garage", gx, gy, 0.65, 4.6, 4.6, 1.3, color=(0.55, 0.55, 0.5)))
    grx, gry = tz["grass"]
    parts.append(flat_patch("garden_grass", grx, gry, 8, 12, (0.2, 0.5, 0.2), z=0.02))
    car_colors = [(0.75, 0.15, 0.15), (0.15, 0.25, 0.6), (0.8, 0.8, 0.82), (0.2, 0.2, 0.22)]
    for i, (cx, cy, cyaw) in enumerate(tz["cars"]):
        parts.append(car(f"car_{i}", cx, cy, cyaw, car_colors[i % len(car_colors)]))
    for i, (cx, cy) in enumerate(tz["cones"]):
        parts.append(cone(f"cone_{i}", cx, cy))
    for i, t in enumerate(tz["trees"]):
        if args.mesh:
            parts.append(mesh_tree(f"target_tree_{i}", t["x"], t["y"], t["mesh_variant"],
                                    target_h=t["trunk_h"] + t["canopy_r"] * 1.4))
        else:
            parts.append(tree(f"target_tree_{i}", t["x"], t["y"], t["trunk_h"], t["canopy_r"], t["green"]))
    for i, (lx, ly) in enumerate(tz["lamps"]):
        parts.append(lamp_post(f"target_lamp_{i}", lx, ly))
    for i, (px, py) in enumerate(tz["persons"]):
        parts.append(person(f"person_{i+1}", px, py, static=True,
                             color=(0.2, 0.3, 0.6) if i == 0 else (0.6, 0.2, 0.25)))

    # 침입 보행자 + 목적지 마커 (mission_controller.py INTRUDER_WAIT_XY와 반드시 일치)
    parts.append('\n    <!-- 침입 보행자 (mission_controller.py INTRUDER_WAIT_XY와 좌표 일치) -->\n')
    parts.append(person("person_intruder", INTRUDER_WAIT_XY[0], INTRUDER_WAIT_XY[1],
                         static=False, color=(0.9, 0.55, 0.05)))
    parts.append('\n    <!-- 배송 목적지 -->\n')
    parts.append(f'    <model name="delivery_target_marker"><static>true</static>'
                  f'<pose>{tx:.1f} {ty:.1f} 0.12 0 0 0</pose><link name="l">'
                  f'<visual name="v"><geometry><sphere><radius>0.12</radius></sphere></geometry>'
                  f'<material><ambient>0.1 0.85 0.2 0.8</ambient></material></visual></link></model>\n')

    # 드론 (불변)
    parts.append('\n')
    parts.append(DRONE_TEMPLATE.format(sx=sx, sy=sy, sz=DRONE_SPAWN_Z))

    parts.append('\n  </world>\n</sdf>\n')
    return "".join(parts)


def validate_xml(xml_text, label):
    import xml.dom.minidom as minidom
    try:
        minidom.parseString(xml_text)
    except Exception as e:
        raise SystemExit(f"[생성 실패] {label}: SDF가 well-formed XML이 아님 — {e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=0, help="랜덤 시드 (기본 0)")
    ap.add_argument("--batch", type=int, default=None,
                     help="지정하면 seed 0..N-1 로 N개 생성 + manifest.json 기록")
    ap.add_argument("--out", default=None, help="출력 경로 (기본: generated/residential_delivery_seed{N}.world)")
    ap.add_argument("--outdir", default="generated", help="배치 생성 시 출력 디렉토리 (worlds/ 기준 상대경로)")
    ap.add_argument("--n-buildings", type=int, default=DEFAULT_ARGS["n_buildings"],
                     help="회랑 배경 건물 개수 (기본 40, '집 많게')")
    ap.add_argument("--n-gates", type=int, default=DEFAULT_ARGS["n_gates"], help="시케인 게이트 개수 (기본 4)")
    ap.add_argument("--lane-offset", type=float, default=DEFAULT_ARGS["lane_offset"],
                     help="도로 바깥쪽 건물 시작 y (기본 11m)")
    ap.add_argument("--half-width", type=float, default=DEFAULT_ARGS["half_width"],
                     help="건물 배치 최대 반폭 (기본 40m, '넓게')")
    ap.add_argument("--max-height", type=float, default=DEFAULT_ARGS["max_height"],
                     help="타워 최대 높이 (기본 90m, '높이 제각각')")
    ap.add_argument("--target-jitter", type=float, default=DEFAULT_ARGS["target_jitter"],
                     help="목적지 채점 시나리오 위치 지터(m). 0=원본과 동일 배치 (기본 0)")
    ap.add_argument("--mesh", action="store_true",
                     help="저층주택/가로수를 Kenney City Kit Suburban 메시로 (협곡 타워는 박스 유지)")
    args = ap.parse_args()

    if args.mesh and not os.path.isdir(KENNEY_DIR):
        raise SystemExit(f"--mesh: {KENNEY_DIR} 없음 — Kenney City Kit Suburban 에셋이 안 들어있음")

    seeds = list(range(args.batch)) if args.batch is not None else [args.seed]
    manifest = {"target_xy": list(TARGET_XY), "start_xy": list(START_XY), "worlds": []}

    for s in seeds:
        xml_text = build_world_xml(args, s)
        validate_xml(xml_text, f"seed {s}")
        if args.out and args.batch is None:
            out_path = args.out
        else:
            out_dir = os.path.join(HERE, args.outdir)
            os.makedirs(out_dir, exist_ok=True)
            out_path = os.path.join(out_dir, f"residential_delivery_seed{s}.world")
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(xml_text)
        print(f"[ok] seed={s} → {out_path} ({len(xml_text)} bytes)")
        manifest["worlds"].append({"seed": s, "path": os.path.relpath(out_path, HERE)})

    if args.batch is not None:
        manifest_path = os.path.join(HERE, args.outdir, "manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(manifest, f, ensure_ascii=False, indent=2)
        print(f"[ok] manifest → {manifest_path}  (남은작업 #4 성공률 실험에서 사용)")


if __name__ == "__main__":
    main()
