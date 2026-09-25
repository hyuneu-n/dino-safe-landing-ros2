#!/usr/bin/env python3
"""Build the V2 capstone deck using the supplied HTML deck's visual system.

The reference file is used only for its CSS. Project copy and slide contents are
maintained here so the generated presentation remains evidence-based.
"""
from __future__ import annotations

import base64
import html
import mimetypes
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DEFAULT_TEMPLATE = Path("/mnt/c/Users/hyuneun/Downloads/36_발표_시간축과confound.html")


def data_uri(relative: str) -> str:
    path = ROOT / relative
    mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    return f"data:{mime};base64,{base64.b64encode(path.read_bytes()).decode()}"


def fig(relative: str, caption: str = "", cls: str = "") -> str:
    cap = f'<div class="cap">{caption}</div>' if caption else ""
    return f'<div class="fig {cls}"><img src="{data_uri(relative)}">{cap}</div>'


def slide(number: int, category: str, title: str, subtitle: str, tagline: str, body: str) -> str:
    return f"""
<div class="slide" id="slide-{number}">
  <div class="s-header">
    <div class="s-num">{number:02d}</div>
    <div class="s-title-block">
      <div class="s-category">{category}</div>
      <div class="s-title">{title}</div>
      <div class="s-subtitle">{subtitle}</div>
    </div>
  </div>
  <div class="s-tagline">{tagline}</div>
  <div class="s-body">{body}</div>
</div>"""


def part(index: int, title: str, subtitle: str) -> str:
    return f"""
<div class="slide part-frame">
  <div class="part-slide">
    <div class="pnum">PART {index}</div>
    <div class="ptitle">{title}</div>
    <div class="psub">{subtitle}</div>
  </div>
</div>"""


def metric(value: str, unit: str, label: str) -> str:
    return f'<div class="card metric-card"><div class="metric"><div class="val">{value}</div><div class="unit">{unit}</div></div><div class="metric-label">{label}</div></div>'


def build(template: Path = DEFAULT_TEMPLATE) -> Path:
    source = template.read_text(encoding="utf-8")
    css_match = re.search(r"<style>(.*?)</style>", source, re.S)
    if not css_match:
        raise RuntimeError(f"CSS not found in template: {template}")
    css = css_match.group(1)
    css += """
/* safe_landing V2 additions */
.slide{height:1080px;min-height:1080px}
.part-frame{border:none}
.tech-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:18px;width:100%}
.tech-item{border:1px solid var(--border);border-radius:8px;padding:20px;background:#fff;min-height:178px}
.tech-name{font-size:22px;font-weight:900;color:var(--navy);margin-bottom:8px}
.tech-role{font-size:17px;line-height:1.55;color:var(--text)}
.metric-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:18px;width:100%}
.metric-card{padding:23px 24px;background:var(--sky)}
.metric-label{font-size:17px;color:var(--navy);font-weight:700;margin-top:10px}
.status-grid{display:grid;grid-template-columns:240px 1fr 1fr;gap:1px;background:var(--border);border:1px solid var(--border);border-radius:8px;overflow:hidden;width:100%}
.status-grid>div{background:white;padding:13px 18px;font-size:17px;line-height:1.42}
.status-grid .head{background:var(--navy);color:#fff;font-weight:700}
.status-grid .key{font-weight:800;color:var(--navy)}
.status-grid .done{color:var(--green);font-weight:700}.status-grid .todo{color:var(--orange);font-weight:700}
.photo-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:15px;width:100%}
.photo-card{border:1px solid var(--border);border-radius:7px;overflow:hidden;background:#fff}
.photo-card img{display:block;width:100%;height:275px;object-fit:cover}
.photo-card .photo-cap{padding:10px 14px;font-size:16px;line-height:1.4;color:var(--navy);font-weight:700}
.journey{display:grid;grid-template-columns:repeat(7,1fr);gap:9px;width:100%;align-items:stretch}
.journey-step{border:2px solid var(--blue);border-radius:8px;padding:18px 10px;text-align:center;background:#fff;min-height:180px;display:flex;flex-direction:column;justify-content:center}
.journey-step .n{font-size:15px;color:var(--blue);font-weight:900;margin-bottom:8px}
.journey-step .t{font-size:19px;color:var(--navy);font-weight:900;line-height:1.3}
.journey-step .d{font-size:14px;color:var(--muted);line-height:1.4;margin-top:8px}
.dashboard{display:grid;grid-template-columns:1.1fr 1.8fr 1fr;grid-template-rows:430px 125px;gap:12px;width:100%;height:567px}
.dash-panel{border:2px solid var(--navy);border-radius:8px;padding:16px;background:#f8fafc}
.dash-panel h3{font-size:20px;color:var(--navy);margin-bottom:10px}.dash-panel p{font-size:16px;line-height:1.5}
.dash-controls{grid-column:1/4;display:grid;grid-template-columns:repeat(4,1fr);gap:12px}
.dash-button{border-radius:7px;background:var(--navy);color:#fff;font-size:18px;font-weight:800;display:flex;align-items:center;justify-content:center}
.roadmap{display:grid;grid-template-columns:repeat(5,1fr);gap:12px;width:100%}
.road-stage{border-top:8px solid var(--blue);background:#f8fafc;padding:20px 17px;border-radius:5px;min-height:300px}
.road-stage .rnum{font-size:15px;color:var(--blue);font-weight:900}.road-stage .rtitle{font-size:21px;font-weight:900;color:var(--navy);margin:10px 0}.road-stage .rbody{font-size:16px;line-height:1.55;color:var(--text)}
.footnote{position:absolute;left:44px;bottom:18px;font-size:13px;color:var(--muted)}
.nav{position:fixed;right:22px;bottom:22px;z-index:50;display:flex;gap:8px}.nav button{border:none;border-radius:7px;background:var(--navy);color:#fff;padding:10px 16px;font-size:16px;cursor:pointer;box-shadow:0 2px 8px rgba(0,0,0,.25)}
.title-meta{font-size:18px;color:rgba(255,255,255,.72);line-height:1.8}
@media print{.nav{display:none}.slide{height:1080px;min-height:1080px}}
"""

    images = {
        "freight": data_uri("docs/images/metropolis/freight_yard_down.png"),
        "downtown": data_uri("docs/images/metropolis/downtown.png"),
        "industrial": data_uri("docs/images/metropolis/industrial.png"),
        "interchange": data_uri("docs/images/metropolis/interchange.png"),
        "mountain": data_uri("docs/images/metropolis/mountain_town.png"),
        "logistics": data_uri("docs/images/metropolis/logistics.png"),
        "campus": data_uri("docs/images/metropolis/campus.png"),
        "mcd": data_uri("docs/images/metropolis/mcdonalds.png"),
        "subway": data_uri("docs/images/metropolis/subway.png"),
        "eval_input": data_uri("docs/presentation/source_images/seed0_rgb.png"),
        "eval_pick": data_uri("docs/presentation/source_images/seed0_pick.png"),
    }

    slides: list[str] = []
    slides.append(f"""
<div class="slide">
  <div class="title-slide">
    <div style="position:relative;z-index:1">
      <div style="display:inline-block;border:1.5px solid rgba(255,255,255,.4);border-radius:20px;padding:7px 20px;font-size:13px;color:rgba(255,255,255,.7);font-weight:500;letter-spacing:2px;margin-bottom:36px">CAPSTONE DESIGN · PROJECT REVIEW · 2026</div>
      <div style="font-size:62px;font-weight:900;color:#fff;line-height:1.15;letter-spacing:-2px;margin-bottom:18px">RGB-D 기반 배송 드론<br>안전 착륙 시스템</div>
      <div style="font-size:27px;color:#93c5fd;font-weight:500;margin-bottom:34px">목적지 선택부터 장애물 회피·안전 착륙까지의 참여형 시뮬레이션</div>
      <div style="width:90px;height:4px;background:#93c5fd;margin-bottom:32px"></div>
      <div style="font-size:24px;font-weight:700;color:#fff;margin-bottom:8px">[이름 입력]</div>
      <div class="title-meta">서경대학교 컴퓨터공학과 · 종합설계<br>구현 상태 기준 2026.09.16</div>
      <div style="margin-top:34px;padding-top:20px;border-top:1px solid rgba(255,255,255,.2);font-size:16px;color:rgba(255,255,255,.55)">ROS 2 Humble · Gazebo Classic 11 · DINOv2 · RGB-D Vision</div>
    </div>
  </div>
</div>""")

    slides.append(slide(1, "Opening · Problem", "목적지 좌표와 실제 착륙점은 다를 수 있다",
        "좌표는 어디로 갈지는 알려주지만, 그 자리가 지금 비어 있는지는 알려주지 않는다.",
        '이 프로젝트의 질문: <span class="acc">드론이 현재 장면을 보고 어디에 내려야 하는가?</span>',
        f"""
<div class="col col-half">{fig('docs/images/metropolis/freight_yard_down.png','실제 Gazebo 하강 카메라: 화물·팔레트·트럭이 섞인 배송 지점','wide')}</div>
<div class="col col-half">
  <div class="card blue"><div class="card-label">GPS 목적지</div><p>배송 요청이 가리키는 <b>거친 도착 좌표</b></p></div>
  <div class="card red"><div class="card-label">현장의 변화</div><p>사람·차량·화물이 지정 위치를 점유할 수 있음</p></div>
  <div class="card green"><div class="card-label">필요한 동작</div><p>주변의 빈 공간을 선택하거나, 안전하지 않으면 <b>착륙을 보류</b></p></div>
</div>"""))

    slides.append(part(1, "내가 하고자 하는 것", "좌표 도달을 넘어, 현재 장면에 맞춰 안전 착륙점을 선택하는 배송 드론"))

    slides.append(slide(2, "Part 1 · Objective", "입력과 출력이 명확한 안전 착륙 문제",
        "배송 목적지와 RGB-D 센서로 착륙 후보를 판단하고, 안전하지 않으면 내리지 않는 시스템을 목표로 한다.",
        '<span class="acc">지정 좌표에 고집하지 않고</span> 목적지 근처의 안전 공간을 선택한다.',
        """
<div class="col col-1" style="width:100%">
  <div class="pipe" style="justify-content:center;margin-top:45px">
    <div class="pnode sky">배송 요청<small>목적지 좌표</small></div><div class="parr"></div>
    <div class="pnode blue">현장 관측<small>RGB + depth</small></div><div class="parr"></div>
    <div class="pnode">안전 공간 판단<small>장애물·평탄도·여유</small></div><div class="parr"></div>
    <div class="pnode green">의사결정<small>착륙 / 재탐색 / 대기</small></div>
  </div>
  <div class="metric-grid" style="margin-top:55px">
    {metric('RGB','appearance','시각 특징으로 표면 차이')}
    {metric('D','geometry','깊이로 높이·장애물')}
    {metric('1.0','m footprint','드론이 들어갈 여유')}
    {metric('0','unsafe action','안전 후보가 없으면 하강 금지')}
  </div>
</div>"""))

    slides.append(slide(3, "Part 1 · Scope", "연구 범위와 현재 시뮬레이션의 경계",
        "착륙 판단과 상황 대응을 연구하고, 위치 추정·비행 동역학은 현재 범위에서 분리한다.",
        '정확한 표현은 <span class="acc">RGB-D 기반 장애물 판단과 안전 착륙점 선택 시뮬레이션</span>이다.',
        """
<div class="col col-half">
  <div class="sec-h">현재 포함</div>
  <div class="card green"><div class="card-label">Perception</div><p>하강 RGB-D로 장애물·안전 후보 판단</p></div>
  <div class="card green"><div class="card-label">Mission</div><p>이륙 → 이동 → 스캔 → 하강 → 착륙 상태머신</p></div>
  <div class="card green"><div class="card-label">Visualization</div><p>후보·선택점·카메라·미션 상태 표시</p></div>
</div>
<div class="col col-half">
  <div class="sec-h">현재 별도 범위</div>
  <div class="card"><div class="card-label">Localization</div><p>현재 자기 위치는 Gazebo odom 사용 — VIO/SLAM 아님</p></div>
  <div class="card"><div class="card-label">Flight Dynamics</div><p><code>set_entity_state</code>로 위치를 직접 지정하는 운동학적 제어</p></div>
  <div class="card"><div class="card-label">Real Vehicle</div><p>실기체와 실제 비행제어기 연동은 아직 수행하지 않음</p></div>
</div>"""))

    slides.append(part(2, "기술 스택과 현재 구현", "센서 입력부터 착륙점 선택·미션 제어·시각화까지 어떤 코드가 연결되어 있는가"))

    tech = [
        ("ROS 2 Humble", "센서·탐지·미션 노드를 토픽과 서비스로 연결"),
        ("Gazebo Classic 11", "도시·드론·RGB/depth 카메라·장애물 시뮬레이션"),
        ("DINOv2 ViT-S/14", "14×14 패치마다 384차원 특징을 추출"),
        ("PyTorch", "DINO 추론, PCA, 특징 K-means 연산"),
        ("NumPy + Pillow", "depth pooling, 마스크, 연결요소, 좌표 처리"),
        ("RViz2", "RGB·depth·PCA·후보·선택점·미션 상태 시각화"),
        ("Python SDF Generator", "도시·목적지·waypoint·metadata 생성"),
        ("COLLADA / SDF", "건물·지형의 시각 메시와 충돌 지오메트리"),
        ("FFmpeg + 검증 스크립트", "환경 캡처·depth 회귀 검사·결과 기록"),
    ]
    tech_html = ''.join(f'<div class="tech-item"><div class="tech-name">{a}</div><div class="tech-role">{b}</div></div>' for a,b in tech)
    slides.append(slide(4, "Part 2 · Technology Stack", "기술 스택은 역할별로 분리되어 있다",
        "시뮬레이터·비전 모델·ROS 통신·시각화·절차생성 환경을 하나의 파이프라인으로 연결했다.",
        '핵심은 모델 하나가 아니라 <span class="acc">센서 → 판단 → 제어 → 설명</span>의 연결이다.',
        f'<div class="col col-1" style="width:100%"><div class="tech-grid">{tech_html}</div></div>'))

    slides.append(slide(5, "Part 2 · Architecture", "현재 시스템 구조 — 착륙 판단과 이동 회피를 분리",
        "하강 카메라는 착륙점을 고르고, 전방 depth는 순항 중 가까운 구조물에 반응한다.",
        '<span class="acc">landing_detector</span>와 <span class="good">mission_controller</span>가 ROS 메시로 연결된다.',
        """
<div class="col col-1" style="width:100%">
  <div class="mflow" style="margin-top:30px">
    <div class="mnode">하강 RGB<small>640×480</small></div><div class="marr"></div>
    <div class="mnode">하강 depth<small>320×240</small></div><div class="marr"></div>
    <div class="mnode b">landing_detector.py<small>DINOv2 + geometry</small></div><div class="marr"></div>
    <div class="mnode g">/landing/target<small>world 좌표</small></div>
  </div>
  <div class="mflow" style="margin-top:52px">
    <div class="mnode">waypoints<small>manifest JSON</small></div><div class="marr"></div>
    <div class="mnode">전방 depth<small>좌 / 중 / 우</small></div><div class="marr"></div>
    <div class="mnode b">mission_controller.py<small>상태머신 + 회피</small></div><div class="marr"></div>
    <div class="mnode g">Gazebo drone<small>pose service</small></div>
  </div>
  <div class="card blue" style="margin-top:55px"><div class="card-label">동시에 보이는 결과</div><p style="margin:0">RViz: RGB 오버레이 · DINO PCA · 후보 마커 · 최종 착륙점 · point cloud · mission status</p></div>
</div>"""))

    pipeline = [
        ("01", "DINO 특징", "448×448 입력\n14×14 패치\n384차원"),
        ("02", "표면 군집", "cosine 정규화\nK-means k=5"),
        ("03", "깊이 장애물", "지면 중앙값\n0.45m margin"),
        ("04", "안전 마스크", "평탄 군집\n주 표면 색 유사"),
        ("05", "Footprint", "1.0m 반경만큼\n마스크 침식"),
        ("06", "후보 점수", "근접·평탄·여유\n면적 가중합"),
        ("07", "World 좌표", "카메라 내부값\n+ Gazebo odom"),
    ]
    journey = ''.join(f'<div class="journey-step"><div class="n">{n}</div><div class="t">{t}</div><div class="d">{d.replace(chr(10),"<br>")}</div></div>' for n,t,d in pipeline)
    slides.append(slide(6, "Part 2 · Landing Perception", "착륙점 판단 알고리즘 — 구현된 처리 순서",
        "시각적 표면 차이와 기하학적 높이 차이를 결합한 뒤, 드론이 들어갈 여유까지 확인한다.",
        '점수 = <span class="acc">0.40 근접 + 0.20 평탄 + 0.25 여유 + 0.15 면적</span>',
        f"""
<div class="col col-1" style="width:100%">
  <div class="journey" style="margin-top:34px">{journey}</div>
  <div class="card red" style="margin-top:48px"><div class="card-label">중요한 해석</div><p style="margin:0">DINO 군집은 사람·차량의 이름을 출력하는 객체 검출이 아니다. <b>처음 보는 장면의 표면 특징을 무라벨로 나누고</b>, depth가 실제 장애물 높이와 공간 여유를 보완한다.</p></div>
</div>"""))

    slides.append(slide(7, "Part 2 · Mission Control", "미션·전방 회피·하강 중 재평가",
        "상태머신은 구현되어 있지만, 도시 어디서나 경로를 재탐색하는 완전한 항법기는 아니다.",
        '<span class="acc">TAKEOFF → CRUISE → ARRIVE → SCAN → DESCEND → LANDED</span>',
        """
<div class="col col-half">
  <div class="sec-h">이동 중</div>
  <div class="card blue"><div class="card-label">Waypoint</div><p>manifest의 경유점을 순서대로 통과, 22m 고도·8m/s</p></div>
  <div class="card blue"><div class="card-label">Reactive avoidance</div><p>전방 depth의 좌/중/우 5% 거리 비교 → 넓은 쪽으로 최대 9m 측면 이동</p></div>
  <div class="card"><div class="card-label">현재 한계</div><p>감속·정지·막힘 탈출·전역 경로 재탐색은 아직 없음</p></div>
</div>
<div class="col col-half">
  <div class="sec-h">도착·하강 중</div>
  <div class="card green"><div class="card-label">SCAN</div><p>12m에서 8초간 선택점을 수집하고 좌표 중앙값 사용</p></div>
  <div class="card green"><div class="card-label">Re-alignment</div><p>하강 중 새 선택점이 0.7m 이상 이동하면 착륙점 재정렬</p></div>
  <div class="card"><div class="card-label">Dynamic event</div><p>11.5m 아래에서 데모용 사람이 착륙 예정지로 이동</p></div>
</div>"""))

    status_rows = [
        ("도시·센서 환경", "배치, RGB/depth, 산길·캠퍼스 진입로 검사", "환경 제작 종료", "done"),
        ("착륙점 탐지", "DINOv2+depth, 후보·선택점·오버레이", "안전 실패 처리 보강", "done"),
        ("기본 미션", "이륙·경유점·스캔·하강 상태머신", "새 도시 종단 간 성공률 미측정", "todo"),
        ("전방 회피", "좌/중/우 depth 기반 측면 회피", "정지·탈출·경로 복구 미완성", "todo"),
        ("동적 장애물", "하강 중 사람 자동 진입 이벤트", "자연스러운 보행·사용자 버튼 미완성", "todo"),
        ("대시보드", "RViz 기술 시각화", "일반인용 UI·반복 배송 미구현", "todo"),
    ]
    rows = ''.join(f'<div class="key">{a}</div><div>{b}</div><div class="{cls}">{c}</div>' for a,b,c,cls in status_rows)
    slides.append(slide(8, "Part 2 · Implementation Status", "현재 기술을 어디까지 구현했는가",
        "구현된 코드, 검증한 범위, 다음에 필요한 작업을 한 문장에 섞지 않는다.",
        '<span class="good">환경·센서와 착륙 판단 프로토타입 확보</span> · <span class="warn">도시 전체 배송 신뢰성은 아직 미측정</span>',
        f"""
<div class="col col-1" style="width:100%">
  <div class="status-grid">
    <div class="head">영역</div><div class="head">현재 구현·검증</div><div class="head">남은 기술 과제</div>
    {rows}
  </div>
</div>"""))

    slides.append(slide(9, "Part 2 · Existing Evaluation", "이전 정지 장면의 착륙점 선택 비교",
        "동일한 Gazebo 장면·GT·드론 footprint 조건에서 세 방법의 선택점을 비교했다.",
        'DINOv2 + depth: <span class="good">22/24</span> · depth-only: <span class="warn">17/24</span> · RGB-only: <span class="warn">11/24</span>',
        f"""
<div class="col col-half">
  <div class="photo-grid" style="grid-template-columns:1fr 1fr">
    <div class="photo-card"><img src="{images['eval_input']}"><div class="photo-cap">저장된 seed 0 입력 RGB</div></div>
    <div class="photo-card"><img src="{images['eval_pick']}"><div class="photo-cap">JSON에 기록된 융합 방식 선택점</div></div>
  </div>
  <div class="card" style="margin-top:15px"><p style="margin:0">성공 기준: 선택점 중심 1.0m footprint 안에 GT 장애물이 없음</p></div>
</div>
<div class="col col-half">
  <table>
    <thead><tr><th>방법</th><th>후보 제시</th><th>안전점 선택</th><th>평균 MOD</th></tr></thead>
    <tbody>
      <tr class="best"><td>DINOv2 + depth</td><td>24/24</td><td>22/24</td><td>3.69m</td></tr>
      <tr><td>depth-only 재구현</td><td>24/24</td><td>17/24</td><td>2.22m</td></tr>
      <tr><td>OpenLander RGB-only</td><td>20/24</td><td>11/24</td><td>2.63m</td></tr>
    </tbody>
  </table>
  <div class="card red" style="margin-top:24px"><div class="card-label">해석 범위</div><p style="margin:0">이전 단일 회랑의 <b>정지 한 프레임 평가</b>다. 현재 확장 도시의 실제 비행·하강·접촉까지 포함한 배송 성공률이 아니다.</p></div>
</div>"""))

    slides.append(part(3, "현재 맵 — 기술을 시험하는 환경", "맵 소개가 목적이 아니라, 서로 다른 장애물·표면·고도 조건을 제공하는 시험장"))

    map_cards = [
        (images['downtown'], "도심", "고층 구조물·좁은 교차로"),
        (images['industrial'], "산업·물류", "트럭·컨테이너·적재물"),
        (images['interchange'], "강·고가도로", "상부 구조·교각·높이 차이"),
        (images['mountain'], "산동네", "경사면·테라스·굽은 도로"),
    ]
    cards = ''.join(f'<div class="photo-card"><img src="{src}"><div class="photo-cap">{title} — {desc}</div></div>' for src,title,desc in map_cards)
    slides.append(slide(10, "Part 3 · Environment", "하나의 맵 안에 서로 다른 시험 조건",
        "약 2.5×1.9km의 절차생성 환경을 만들고, 시각 메시와 충돌 지오메트리를 함께 검증했다.",
        'seed 0 기준 <span class="acc">건물 148동 · 배송 목적지 22곳 · 지정 위치 점유 상황 4곳</span>',
        f'<div class="col col-1" style="width:100%"><div class="photo-grid">{cards}</div></div>'))

    dest_cards = [
        (images['logistics'], "SkyDrop 물류창고", "드론 실제 생성·출발 위치"),
        (images['campus'], "서경대학교", "사진 특징을 반영한 높이 12m 캠퍼스"),
        (images['mcd'], "생활 목적지", "상점·차양·차량이 있는 배송 공간"),
        (images['freight'], "Foundry Yard", "지정 좌표의 화물을 피해 빈 바닥 선택"),
    ]
    dest = ''.join(f'<div class="photo-card"><img src="{src}"><div class="photo-cap">{title} — {desc}</div></div>' for src,title,desc in dest_cards)
    slides.append(slide(11, "Part 3 · Destinations", "배송 시나리오가 보이는 목적지",
        "물류창고에서 출발해 상점·광장·공장·학교로 배송하며 서로 다른 착륙 조건을 만든다.",
        '높이 12·34·36·96m의 <span class="warn">4개 목적지는 고도 처리 보강 후 자동 비행</span>',
        f'<div class="col col-1" style="width:100%"><div class="photo-grid">{dest}</div></div>'))

    slides.append(part(4, "최종 졸업작품 시나리오", "관람객이 배송지를 고르고 상황을 바꾸면, 드론의 판단 과정과 결과가 화면에 보이는 체험"))

    final_steps = [
        ("01","목적지 선택","장소 카드와 지도"),("02","배송 시작","물류창고 이륙"),("03","이동 판단","전방 카메라·회피"),
        ("04","현장 스캔","하강 RGB-D"),("05","공간 선택","요청점 대신 빈 곳"),("06","관람객 개입","사람·장애물 추가"),("07","결과","착륙 또는 안전 대기")
    ]
    final_html = ''.join(f'<div class="journey-step"><div class="n">{n}</div><div class="t">{t}</div><div class="d">{d}</div></div>' for n,t,d in final_steps)
    slides.append(slide(12, "Part 4 · User Journey", "관람객이 경험하는 최종 결과물",
        "직접 조종하는 게임보다, 드론에게 배송을 맡기고 상황을 바꾸는 참여형 관제 체험으로 설계한다.",
        '핵심 경험: <span class="acc">내가 고른 목적지</span>에서 <span class="good">드론의 판단이 상황에 따라 바뀌는 것</span>',
        f"""
<div class="col col-1" style="width:100%">
  <div class="journey" style="margin-top:32px">{final_html}</div>
  <div class="card blue" style="margin-top:50px"><p style="margin:0">안전 공간이 사라졌다면 <b>기다리는 것도 올바른 결과</b>로 표시한다. 개입 버튼은 환경만 바꾸며, 장애물 좌표나 정답 착륙점을 제어기에 전달하지 않는다.</p></div>
</div>"""))

    slides.append(slide(13, "Part 4 · Dashboard", "최종 화면 구성 — 판단 이유가 보이는 관제 UI",
        "일반인은 RGB 화면과 한 문장 상태를 먼저 보고, depth·DINO PCA는 기술 상세에서 확인한다.",
        '<span class="acc">목적지 → 현재 장면 → 판단 이유 → 실제 결과</span>가 한 화면에서 이어져야 한다.',
        """
<div class="col col-1" style="width:100%">
  <div class="dashboard">
    <div class="dash-panel"><h3>지도 · 목적지</h3><p>물류 야드 / 상점 / 광장 / 학교</p><p>현재 위치와 실제 이동 궤적</p></div>
    <div class="dash-panel"><h3>큰 카메라 화면</h3><p>전방 RGB 또는 하강 RGB</p><p>장애물 제외 영역 · 후보 영역 · 요청점 · 최종 선택점</p><p style="margin-top:80px;text-align:center;color:var(--blue);font-weight:800">“착륙 예정 위치에 사람이 진입하여 재탐색합니다.”</p></div>
    <div class="dash-panel"><h3>미션 상태 · 결과</h3><p>이동 / 정지 / 우회 / 스캔 / 재탐색 / 착륙 / 보류</p><p>접촉 여부 · 요청점 거리 · 재선택 횟수</p></div>
    <div class="dash-controls">
      <div class="dash-button">배송 시작</div><div class="dash-button">사람 지나가기</div><div class="dash-button">장애물 추가</div><div class="dash-button">초기화</div>
    </div>
  </div>
</div>"""))

    slides.append(slide(14, "Part 4 · Representative Demo", "대표 시연 — 착륙 예정지에 사람이 들어온다면",
        "고정 영상을 재생하는 대신, 관람객의 개입 후 실제 센서 입력에 따라 결과가 달라지는 장면을 만든다.",
        '목표 동작: <span class="warn">하강 중지</span> → <span class="acc">재탐색</span> → <span class="good">다른 곳 선택 또는 안전 대기</span>',
        """
<div class="col col-half">
  <div class="sec-h">시나리오</div>
  <div class="card blue"><div class="card-label">1. 출발</div><p>SkyDrop 물류창고에서 배송 시작</p></div>
  <div class="card blue"><div class="card-label">2. 도착</div><p>Foundry Yard 또는 Civic Plaza 상공 스캔</p></div>
  <div class="card blue"><div class="card-label">3. 1차 판단</div><p>화물·사람이 있는 요청점을 피해 빈 공간 선택</p></div>
</div>
<div class="col col-half">
  <div class="sec-h">관람객 개입 이후</div>
  <div class="card red"><div class="card-label">4. 사람 진입</div><p>버튼을 눌러 착륙 예정지를 가로지르게 함</p></div>
  <div class="card green"><div class="card-label">5. 안전 반응</div><p>하강 중지·재탐색·재선택 또는 대기</p></div>
  <div class="card"><div class="card-label">6. 결과 카드</div><p>접촉 여부, 요청점과 착륙점 거리, 재선택 횟수</p></div>
</div>"""))

    slides.append(part(5, "앞으로 발전시킬 기술", "맵 확장이 아니라 실패 시 안전 동작·고도 처리·경로 복구·반복 평가에 집중"))

    priorities = [
        ("01","후보 없음","하강 금지\nhover / retry / abort"),
        ("02","후보 안전성","마스크 내부 확인\nfootprint 여유 최대점"),
        ("03","고도·회전","surface z와 yaw를\n좌표 변환에 반영"),
        ("04","이동 회피","감속·정지·막힘 탈출\n경로 복구"),
        ("05","객관적 평가","접촉·관통·최소 여유\n시간·보류·실패 기록"),
    ]
    roads = ''.join(f'<div class="road-stage"><div class="rnum">PRIORITY {n}</div><div class="rtitle">{t}</div><div class="rbody">{b.replace(chr(10),"<br>")}</div></div>' for n,t,b in priorities)
    slides.append(slide(15, "Part 5 · Technical Priorities", "이제 발전시킬 기술 — 안전 실패 처리부터",
        "기능을 많이 붙이기보다, 위험한 상황에서 내리지 않는 동작을 먼저 완성한다.",
        '첫 기준: <span class="warn">안전 후보가 없거나 센서가 오래되면 절대로 하강하지 않는다.</span>',
        f'<div class="col col-1" style="width:100%"><div class="roadmap">{roads}</div></div>'))

    stages = [
        ("1","정적 착륙","짧은 접근\n후보 없음 하강 금지"),
        ("2","동적 재평가","사람 진입\n정지·재선택·대기"),
        ("3","이동 회피","실제로 막힌 구간\n정지·복구 검증"),
        ("4","참여형 UI","목적지 선택\n반복 배송·개입"),
        ("5","복층 확장","캠퍼스·교량·산동네\n높이 있는 목적지"),
    ]
    stage_html = ''.join(f'<div class="road-stage"><div class="rnum">STAGE {n}</div><div class="rtitle">{t}</div><div class="rbody">{b.replace(chr(10),"<br>")}</div></div>' for n,t,b in stages)
    slides.append(slide(16, "Part 5 · Development Order", "짧은 코스부터 통과 기준을 쌓는다",
        "각 단계는 코드가 존재하는지가 아니라, 반복 실험에서 안전 조건을 만족하는지로 완료를 판단한다.",
        '성공 지표: <span class="acc">접촉 0 · 관통 0 · 최소 여유 · 재선택/대기 · 시간 초과</span>',
        f"""
<div class="col col-1" style="width:100%">
  <div class="roadmap">{stage_html}</div>
  <div class="card green" style="margin-top:32px"><p style="margin:0">첫 통합 코스 후보: <b>Foundry Yard의 짧은 접근 + 정적 화물 회피</b>. 그다음 Civic Plaza의 사람 개입으로 확장.</p></div>
</div>"""))

    slides.append(slide(17, "Conclusion", "환경은 준비됐다 — 이제 기술의 신뢰성을 증명할 차례",
        "현재 확보한 프로토타입을 안전한 통합 시연으로 발전시키는 단계다.",
        '<span class="good">다양한 환경 + RGB-D 착륙 판단 + 기본 미션</span> → <span class="acc">안전 실패 처리 + 통합 검증 + 관람객 인터랙션</span>',
        """
<div class="col col-half">
  <div class="sec-h">지금까지 확보</div>
  <div class="card green"><p>148동·22개 목적지의 시험 환경</p></div>
  <div class="card green"><p>DINOv2+depth 착륙점 판단</p></div>
  <div class="card green"><p>상태머신·반응형 회피·RViz 시각화</p></div>
</div>
<div class="col col-half">
  <div class="sec-h">다음 회의에서 결정</div>
  <div class="card blue"><div class="card-label">첫 시연 코스</div><p>어디서 어떤 장애물을 보여줄 것인가?</p></div>
  <div class="card blue"><div class="card-label">성공 정의</div><p>착륙·대기·실패를 어떤 지표로 평가할 것인가?</p></div>
  <div class="card dark"><div class="card-label">질의응답</div><p style="font-size:26px;margin:0">감사합니다.</p></div>
</div>"""))

    script = """
<script>
const slides=[...document.querySelectorAll('.slide')]; let current=0;
function go(delta){current=Math.max(0,Math.min(slides.length-1,current+delta));slides[current].scrollIntoView({behavior:'smooth',block:'start'});}
document.addEventListener('keydown',e=>{if(['ArrowRight','ArrowDown','PageDown',' '].includes(e.key)){e.preventDefault();go(1)}if(['ArrowLeft','ArrowUp','PageUp'].includes(e.key)){e.preventDefault();go(-1)}if(e.key==='Home'){current=0;go(0)}if(e.key==='End'){current=slides.length-1;go(0)}});
</script>"""
    document = f"""<!DOCTYPE html><html lang="ko"><head><meta charset="UTF-8"><title>RGB-D 배송 드론 안전 착륙 — 종합설계 발표 V2</title><link rel="preconnect" href="https://fonts.googleapis.com"><link href="https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@300;400;500;700;900&display=swap" rel="stylesheet"><style>{css}</style></head><body><div class="deck">{''.join(slides)}</div><div class="nav"><button onclick="go(-1)">◀ 이전</button><button onclick="go(1)">다음 ▶</button></div>{script}</body></html>"""
    output = OUT / "safe_landing_first_review_v2.html"
    output.write_text(document, encoding="utf-8")
    render_dir = OUT / "render_v2_html"
    render_dir.mkdir(exist_ok=True)
    render_override = """
html,body{margin:0!important;padding:0!important;width:1800px!important;height:1080px!important;overflow:hidden!important;background:#fff!important}
.deck{width:1800px!important;margin:0!important;padding:0!important;gap:0!important}
.slide{margin:0!important}.nav{display:none!important}
"""
    for index, slide_html in enumerate(slides, start=1):
        standalone = f"""<!DOCTYPE html><html lang="ko"><head><meta charset="UTF-8"><title>Slide {index:02d}</title><link rel="preconnect" href="https://fonts.googleapis.com"><link href="https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@300;400;500;700;900&display=swap" rel="stylesheet"><style>{css}{render_override}</style></head><body><div class="deck">{slide_html}</div></body></html>"""
        (render_dir / f"slide_{index:02d}.html").write_text(standalone, encoding="utf-8")

    print(
        f"Created {output} with {len(slides)} slides; no video elements; "
        f"standalone render pages in {render_dir}"
    )
    return output


if __name__ == "__main__":
    build()
