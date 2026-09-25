#!/usr/bin/env python3
"""Build the compact HTML-first capstone deck inspired by the user's last deck."""
from __future__ import annotations

import base64
import mimetypes
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent


def uri(relative: str) -> str:
    path = ROOT / relative
    mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    return f"data:{mime};base64,{base64.b64encode(path.read_bytes()).decode()}"


CSS = r"""
:root{--navy:#071a68;--blue:#0a4ea1;--sky:#eaf3ff;--line:#d5dbe5;--ink:#111827;--muted:#5f6570;--green:#18883b;--red:#c71920;--orange:#df7200}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:#dfe3ea;font-family:"Malgun Gothic","Noto Sans KR",Arial,sans-serif;color:var(--ink)}
.deck{display:flex;flex-direction:column;align-items:center;gap:34px;padding:34px 0 80px}
.slide{position:relative;width:1600px;height:900px;background:#fff;border:10px solid var(--navy);overflow:hidden;padding:24px 42px 30px}
.head{height:122px;padding-left:125px;position:relative}.num{position:absolute;left:-30px;top:-10px;width:110px;height:110px;background:var(--navy);color:#fff;display:flex;align-items:center;justify-content:center;font-size:52px;font-weight:900}
.eyebrow{font-size:16px;color:#777;margin:2px 0 8px}.title{font-size:43px;line-height:1.08;font-weight:900;letter-spacing:-1.8px}.sub{font-size:18px;color:var(--blue);margin-top:5px}
.body{height:700px;padding-top:8px}.statement{background:#f3f3f3;box-shadow:4px 5px 6px #bbb;padding:13px 25px;font-size:21px;font-weight:800;text-align:center;margin:4px 0 20px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:26px}.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}
.section{font-size:23px;font-weight:900;color:var(--navy);padding-bottom:8px;border-bottom:2px solid #eceff3;margin:10px 0 12px}.section.red{color:var(--red)}
.box{border:1px solid var(--line);border-radius:4px;padding:15px 18px;background:#fff}.box.blue{border-left:6px solid var(--blue);background:#f5f9ff}.box.green{border-left:6px solid var(--green);background:#f2fbf4}.box.red{border-left:6px solid var(--red);background:#fff5f5}.box.gray{background:#f5f6f8}
.box h3{font-size:20px;color:var(--navy);margin:0 0 8px}.box p,.box li{font-size:17px;line-height:1.52;margin:3px 0}.box ul{margin:5px 0 0;padding-left:22px}
.bullets{margin:3px 0;padding-left:24px}.bullets li{font-size:18px;line-height:1.58;margin:3px 0}
.accent{color:var(--blue);font-weight:900}.good{color:var(--green);font-weight:900}.bad{color:var(--red);font-weight:900}.warn{color:var(--orange);font-weight:900}.mono{font-family:Consolas,monospace;font-size:.9em}
.flow{display:grid;grid-template-columns:repeat(7,1fr);gap:8px;align-items:stretch}.step{position:relative;border:2px solid #93afd6;border-radius:5px;background:#f8fbff;padding:13px 8px;text-align:center;min-height:107px}.step:not(:last-child):after{content:"›";position:absolute;right:-11px;top:30px;color:var(--blue);font-size:30px;font-weight:900;z-index:2}.step .n{font-size:13px;font-weight:900;color:var(--blue)}.step .t{font-size:18px;font-weight:900;margin:7px 0 3px}.step .d{font-size:14px;color:var(--muted);line-height:1.35}
.pipe{display:grid;grid-template-columns:1fr 42px 1fr 42px 1fr 42px 1fr;align-items:center;gap:5px}.node{border:2px solid var(--blue);border-radius:7px;padding:14px;text-align:center;font-weight:900;font-size:18px;background:#f8fbff;min-height:94px}.node small{display:block;font-weight:400;color:var(--muted);font-size:14px;line-height:1.35;margin-top:7px}.arrow{text-align:center;font-size:33px;color:var(--blue);font-weight:900}
.tagrow{display:flex;flex-wrap:wrap;gap:8px}.tag{padding:7px 11px;border-radius:4px;background:var(--navy);color:#fff;font-size:14px;font-weight:800}.tag.light{background:#e8f1ff;color:var(--blue);border:1px solid #a8c3e9}
.metricrow{display:grid;grid-template-columns:repeat(5,1fr);gap:10px}.metric{background:#f2f5f9;border-top:6px solid var(--blue);padding:13px 14px;min-height:105px}.metric b{display:block;font-size:24px;color:var(--navy);margin-bottom:4px}.metric span{font-size:14px;line-height:1.35;color:var(--muted)}
.table{width:100%;border-collapse:collapse;font-size:16px}.table th{background:var(--navy);color:#fff;text-align:left;padding:10px 12px}.table td{border:1px solid var(--line);padding:9px 12px;line-height:1.35}.table tr:nth-child(even) td{background:#f7f8fa}
.figure{border:1px solid var(--line);background:#f7f7f7;overflow:hidden}.figure img{display:block;width:100%;height:100%;object-fit:cover}.cap{font-size:14px;font-weight:700;padding:7px 10px;background:#fff;color:#202b3c}
.photo4{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:245px 245px;gap:12px}.photo4 .figure{display:grid;grid-template-rows:1fr auto}
.algo{display:grid;grid-template-columns:repeat(4,1fr);gap:9px}.algo .box{padding:12px;min-height:105px}.algo h3{font-size:17px}.algo p{font-size:14px;line-height:1.35}
.statusline{display:flex;align-items:center;justify-content:space-between;margin:3px 0 17px}.state{flex:1;text-align:center;padding:12px 4px;border:2px solid #95afd0;border-radius:22px;margin:0 4px;font-size:14px;font-weight:900;color:var(--navy)}.state.active{border-color:#e39a35;background:#fff5e6}.state.done{border-color:#5da76d;background:#effaf1}
.dash{display:grid;grid-template-columns:1.05fr 1.8fr 1fr;grid-template-rows:250px 72px;gap:9px}.dash>div{border:2px solid var(--navy);padding:13px;background:#f8fafc}.dash h3{font-size:18px;color:var(--navy);margin:0 0 8px}.dash p{font-size:14px;line-height:1.45;margin:3px 0}.controls{grid-column:1/4!important;display:grid;grid-template-columns:repeat(4,1fr);gap:8px;border:none!important;padding:0!important;background:#fff!important}.controls span{display:flex;align-items:center;justify-content:center;background:var(--navy);color:#fff;font-weight:900}
.road{display:grid;grid-template-columns:repeat(5,1fr);gap:10px}.road .box{min-height:170px;border-top:7px solid var(--blue)}.road .k{font-size:13px;color:var(--blue);font-weight:900}.road h3{font-size:19px;margin-top:8px}.road p{font-size:15px}
.note{font-size:14px;color:var(--muted);line-height:1.45}.footer{position:absolute;right:34px;bottom:13px;font-size:12px;color:#a0a5ad}
.nav{position:fixed;right:24px;bottom:22px;display:flex;gap:8px;z-index:20}.nav button{border:0;background:var(--navy);color:white;border-radius:5px;padding:10px 15px;font-size:15px;cursor:pointer;box-shadow:0 2px 8px #777}
.cover{padding:0;background:var(--navy);border-color:var(--navy);color:#fff}.cover-inner{height:100%;position:relative;padding:175px 150px}.cover-kicker{font-size:18px;letter-spacing:4px;color:#9fc5ff;font-weight:700;margin-bottom:35px}.cover-title{font-size:78px;line-height:1;font-weight:900;letter-spacing:-3px;margin-bottom:32px}.cover-line{width:110px;height:6px;background:#78b1ff;margin-bottom:38px}.cover-project{font-size:31px;line-height:1.4;color:#dceaff;font-weight:700}.cover-name{position:absolute;left:150px;bottom:120px;font-size:29px;font-weight:700}.cover-meta{position:absolute;right:150px;bottom:125px;text-align:right;color:#9fc5ff;font-size:17px;line-height:1.7}.cover-orb{position:absolute;border-radius:50%;background:#0a3985}.cover-orb.one{width:360px;height:360px;right:115px;top:80px}.cover-orb.two{width:230px;height:230px;right:390px;bottom:85px;background:#0b4a91}
@media print{body{background:#fff}.deck{padding:0;gap:0}.slide{break-after:page}.nav{display:none}}
"""


def slide(number: int, title: str, subtitle: str, body: str) -> str:
    return f"""<section class="slide" id="slide-{number}"><header class="head"><div class="num">{number:02d}</div><div class="title" style="padding-top:8px">{title}</div><div class="sub">{subtitle}</div></header><main class="body">{body}</main><div class="footer">RGB-D DELIVERY DRONE · 2026.09.16</div></section>"""


def build() -> Path:
    img = {
        "logistics": uri("docs/images/metropolis/logistics.png"),
        "connected": uri("docs/images/metropolis/connected.png"),
        "downtown": uri("docs/images/metropolis/downtown.png"),
        "industrial": uri("docs/images/metropolis/industrial.png"),
        "interchange": uri("docs/images/metropolis/interchange.png"),
        "campus": uri("docs/images/metropolis/campus.png"),
        "eval_rgb": uri("docs/presentation/source_images/seed0_rgb.png"),
        "eval_pick": uri("docs/presentation/source_images/seed0_pick.png"),
    }
    slides: list[str] = []

    slides.append("""
<section class="slide cover">
  <div class="cover-orb one"></div><div class="cover-orb two"></div>
  <div class="cover-inner">
    <div class="cover-kicker">CAPSTONE DESIGN · 2026</div>
    <div class="cover-title">종합설계</div>
    <div class="cover-line"></div>
    <div class="cover-project">RGB-D 기반 배송 드론<br>안전 착륙 시스템</div>
    <div class="cover-name">서현은</div>
    <div class="cover-meta">목적지 선택 · 장애물 판단 · 안전 착륙<br>ROS 2 · Gazebo Classic · DINOv2</div>
  </div>
</section>""")

    slides.append(slide(1, "프로젝트 개요", "좌표 도달 이후, 현재 장면을 보고 안전한 착륙점을 다시 결정하는 배송 드론", f"""
<div class="statement">배송 드론이 목적지 근처에 도착한 뒤, <span class="accent">RGB-D로 주변을 분석해 사람·차량·화물을 피한 빈 공간</span>을 선택한다.</div>
<div class="grid2" style="grid-template-columns:1.08fr .92fr">
  <div>
    <div class="section">왜 필요한가</div>
    <ul class="bullets"><li>배송 좌표는 <b>어디로 갈지</b>만 알려주며, 그 자리가 지금 비어 있는지는 모른다.</li><li>지정 지점이 점유되면 주변의 안전 공간을 고르거나 <b>착륙을 보류</b>해야 한다.</li><li>핵심 질문: <span class="accent">“지금 이 장면에서 어디에 내려야 하는가?”</span></li></ul>
    <div class="section">완성하려는 동작</div>
    <div class="flow">
      <div class="step"><div class="n">01</div><div class="t">목적지</div><div class="d">장소 선택</div></div><div class="step"><div class="n">02</div><div class="t">출발</div><div class="d">물류창고</div></div><div class="step"><div class="n">03</div><div class="t">이동</div><div class="d">전방 depth</div></div><div class="step"><div class="n">04</div><div class="t">스캔</div><div class="d">하강 RGB-D</div></div><div class="step"><div class="n">05</div><div class="t">선택</div><div class="d">빈 공간</div></div><div class="step"><div class="n">06</div><div class="t">재평가</div><div class="d">사람 진입</div></div><div class="step"><div class="n">07</div><div class="t">결과</div><div class="d">착륙/대기</div></div>
    </div>
  </div>
  <div class="figure" style="height:525px"><img src="{img['logistics']}" style="object-fit:cover"><div class="cap">SkyDrop 물류창고 — 배송 드론의 실제 생성·출발 위치</div></div>
</div>"""))

    slides.append(slide(2, "기술 스택과 전체 구조", "착륙 판단과 이동 제어를 분리하고 ROS 2 토픽으로 연결", """
<div class="tagrow" style="margin-bottom:15px"><span class="tag">ROS 2 Humble</span><span class="tag">Gazebo Classic 11</span><span class="tag">Python</span><span class="tag">PyTorch</span><span class="tag">DINOv2 ViT-S/14</span><span class="tag">NumPy · Pillow</span><span class="tag">RViz2</span><span class="tag">SDF · COLLADA</span></div>
<div class="pipe">
  <div class="node">Gazebo 센서<small>하강 RGB·거리 영상<br>전방 거리 · 위치 정보</small></div><div class="arrow">→</div>
  <div class="node">착륙점 판단 노드<small>DINO 특징 + depth<br>후보·최종 착륙점</small></div><div class="arrow">→</div>
  <div class="node">미션 제어 노드<small>상태머신 · 이동 회피<br>하강 중 재정렬</small></div><div class="arrow">→</div>
  <div class="node">Gazebo · RViz<small>드론 이동<br>판단 영역 · 후보 · 궤적</small></div>
</div>
<div class="grid3" style="margin-top:18px">
  <div class="box blue"><h3>착륙 판단 입력</h3><p>하강 카메라 RGB 영상<br>하강 카메라 거리 영상<br>Gazebo의 드론 위치·자세</p></div>
  <div class="box green"><h3>주요 출력</h3><p>최종 안전 착륙점<br>안전 영역·장애물·후보점 시각화</p></div>
  <div class="box gray"><h3>현재 제어 방식</h3><p>Gazebo에서 드론 위치 확인<br>시뮬레이터 내부 위치를 직접 갱신</p></div>
</div>
<div class="statement" style="margin-top:18px;box-shadow:none;border:1px solid var(--line)"><span class="accent">현재 정확한 표현:</span> RGB-D 기반 장애물 판단과 안전 착륙점 선택 시뮬레이션</div>
<div class="grid3" style="margin-top:11px">
  <div class="box green" style="padding:10px 15px"><h3 style="font-size:17px">비전을 사용하는 판단</h3><p style="font-size:14px">착륙 장애물·후보·선택점, 전방 거리 기반 회피</p></div>
  <div class="box red" style="padding:10px 15px"><h3 style="font-size:17px">현재 비전이 아닌 부분</h3><p style="font-size:14px">자기 위치는 Gazebo 제공값 사용, 카메라 기반 위치 추정 미구현</p></div>
  <div class="box gray" style="padding:10px 15px"><h3 style="font-size:17px">현재 실기체가 아닌 부분</h3><p style="font-size:14px">실제 비행제어기·동역학·기체 연동은 범위 밖</p></div>
</div>"""))

    slides.append(slide(3, "착륙점 판단 알고리즘", "DINOv2의 시각적 표면 특징과 depth의 기하 정보를 결합", f"""
<div class="grid2" style="grid-template-columns:.82fr 1.18fr">
  <div class="figure" style="height:535px"><img src="{img['eval_pick']}" style="object-fit:cover"><div class="cap">노란 원: 최종 선택점 · RGB/depth 융합 결과의 저장 예시</div></div>
  <div>
    <div class="algo">
      <div class="box blue"><h3>1. DINO 특징</h3><p>448×448 RGB<br>14×14 patch<br>384차원 특징</p></div>
      <div class="box blue"><h3>2. 표면 군집</h3><p>cosine 정규화<br>K-means <b>k=5</b></p></div>
      <div class="box red"><h3>3. 깊이 장애물</h3><p>지면 중앙값 대비<br><b>0.45m</b> 이상 제거</p></div>
      <div class="box green"><h3>4. 안전 마스크</h3><p>평탄 군집 +<br>주 표면 색 유사</p></div>
      <div class="box green"><h3>5. Footprint</h3><p>드론 반경 <b>1.0m</b><br>들어갈 여유 확인</p></div>
      <div class="box blue"><h3>6. 후보 점수</h3><p>근접 .40 · 평탄 .20<br>여유 .25 · 면적 .15</p></div>
      <div class="box blue"><h3>7. 좌표 변환</h3><p>카메라 정보 +<br>드론 현재 위치</p></div>
      <div class="box gray"><h3>8. 결과 출력</h3><p>지도상의 착륙점<br>후보·판단 영역 시각화</p></div>
    </div>
    <div class="box red" style="margin-top:15px"><h3>해석할 때 주의</h3><p>DINO 군집은 사람·자동차의 이름을 출력하는 객체 검출이 아니다. 처음 보는 장면의 <b>비슷한 표면을 무라벨로 묶고</b>, depth가 실제 높이와 공간 여유를 보완한다.</p></div>
    <div class="box gray" style="margin-top:10px"><p><b>현재 좌표 변환 한계:</b> 드론이 회전하지 않고 지면 높이가 일정하다고 가정한다. 캠퍼스·교량·산동네에서는 실제 표면 높이와 드론 자세를 반영해야 한다.</p></div>
  </div>
</div>"""))

    slides.append(slide(4, "미션 제어와 현재 구현 상태", "기본 흐름은 동작하지만 안전 실패 처리와 경로 복구는 아직 남아 있음", """
<div class="statusline"><div class="state">TAKEOFF</div><div class="state">CRUISE</div><div class="state">ARRIVE</div><div class="state active">SCAN · 8s</div><div class="state done">DESCEND</div><div class="state done">LANDED</div></div>
<table class="table"><thead><tr><th style="width:18%">영역</th><th style="width:42%">현재 구현·확인한 것</th><th>남은 기술 과제</th></tr></thead><tbody>
<tr><td><b>도시·센서</b></td><td>맵 생성, RGB·거리 영상, 충돌 구조, 목적지 목록</td><td><span class="good">환경 제작 종료</span></td></tr>
<tr><td><b>착륙점 탐지</b></td><td>DINOv2와 거리 정보를 결합해 후보·선택점·판단 화면 생성</td><td><span class="bad">후보가 없을 때 하강하는 대체 동작 제거</span></td></tr>
<tr><td><b>기본 미션</b></td><td>이륙→경유점→스캔→하강→착륙 상태머신</td><td><span class="warn">새 도시 종단 간 성공률 미측정</span></td></tr>
<tr><td><b>전방 회피</b></td><td>전방 depth 좌/중/우 여유 비교 후 측면 오프셋</td><td><span class="warn">감속·정지·막힘 탈출·경로 복구</span></td></tr>
<tr><td><b>동적 재평가</b></td><td>하강 중 새 점이 0.7m 이상 이동하면 목표 재정렬</td><td><span class="warn">자연스러운 보행·안전 대기 검증</span></td></tr>
<tr><td><b>최종 UI</b></td><td>RViz 기술 시각화</td><td><span class="warn">목적지 선택·반복 배송·관람객 버튼</span></td></tr>
</tbody></table>
<div class="grid3" style="margin-top:14px"><div class="box blue"><h3>이동 설정</h3><p>순항 고도 22m · 스캔 고도 12m · 속도 8m/s</p></div><div class="box blue"><h3>회피 설정</h3><p>22m 앞까지 확인 · 좌우 9m 범위에서 회피</p></div><div class="box red"><h3>중요한 현재 한계</h3><p>후보가 없어도 목적지 좌표로 내려가는 대체 동작이 존재</p></div></div>"""))

    slides.append(slide(5, "현재까지의 평가와 주장 범위", "기존 정지 장면에서는 융합 방식이 우수했지만 도시 배송 성공률은 아직 별도 측정 필요", f"""
<div class="grid2" style="grid-template-columns:1.1fr .9fr">
  <div>
    <div class="grid2" style="gap:10px"><div class="figure" style="height:315px"><img src="{img['eval_rgb']}"><div class="cap">이전 단일 회랑의 평가 입력 장면</div></div><div class="figure" style="height:315px"><img src="{img['eval_pick']}"><div class="cap">RGB와 거리 정보를 결합해 선택한 실제 지점</div></div></div>
    <div class="box gray" style="margin-top:12px"><p><b>평가 조건:</b> 정지 장면 24장 · 같은 정답 기준 · 선택점 주변의 드론 크기 범위 안에 장애물이 없으면 성공</p></div>
  </div>
  <div>
    <table class="table"><thead><tr><th>방법</th><th>후보 제시</th><th>안전 선택</th><th>목적지와 평균 거리</th></tr></thead><tbody><tr><td><b>DINOv2 + 거리 정보</b></td><td>24/24</td><td class="good">22/24</td><td>3.69m</td></tr><tr><td>거리 정보만 사용</td><td>24/24</td><td>17/24</td><td>2.22m</td></tr><tr><td>RGB만 사용</td><td>20/24</td><td class="bad">11/24</td><td>2.63m</td></tr></tbody></table>
    <div class="box red" style="margin-top:18px"><h3>이 수치가 의미하지 않는 것</h3><ul><li>새 도시 전체 배송 성공률이 아님</li><li>실제 비행·접촉·착륙 동역학 성공률이 아님</li><li>사람 클래스를 인식한 정확도가 아님</li></ul></div>
    <div class="box green" style="margin-top:12px"><h3>다음 평가에서 추가할 지표</h3><p>접촉 0 · 관통 0 · 최소 여유 · 재선택/대기 횟수 · 완료 시간 · 실패 원인</p></div>
  </div>
</div>"""))

    slides.append(slide(6, "현재 시뮬레이션 환경", "맵은 완성된 배경이 아니라 서로 다른 실패 조건을 제공하는 시험장", f"""
<div class="photo4">
  <div class="figure"><img src="{img['downtown']}"><div class="cap">도심 — 건물 사이 협소한 시야와 복잡한 외관</div></div>
  <div class="figure"><img src="{img['industrial']}"><div class="cap">산업지구 — 화물·탱크·창고·불규칙 구조물</div></div>
  <div class="figure"><img src="{img['interchange']}"><div class="cap">강·고가도로 — 교량, 램프, 다층 높이 조건</div></div>
  <div class="figure"><img src="{img['campus']}"><div class="cap">서경대학교 — 경사 진입로, 광장, 고층·아치 건물</div></div>
</div>
<div class="metricrow" style="margin-top:14px"><div class="metric"><b>2.5×1.9km</b><span>전체 환경 규모</span></div><div class="metric"><b>148동</b><span>현재 생성 환경의 건물</span></div><div class="metric"><b>22곳</b><span>배송 목적지</span></div><div class="metric"><b>4곳</b><span>지정 좌표 점유 시나리오</span></div><div class="metric"><b>4곳</b><span>고도 처리 보강 필요</span></div></div>"""))

    slides.append(slide(7, "최종 결과물: 참여형 배송 관제", "직접 조종보다 목적지를 맡기고 상황을 바꾸며 드론의 판단을 확인하는 체험", """
<div class="flow" style="margin-bottom:15px"><div class="step"><div class="n">01</div><div class="t">목적지 선택</div><div class="d">장소 카드·지도</div></div><div class="step"><div class="n">02</div><div class="t">배송 시작</div><div class="d">SkyDrop 창고</div></div><div class="step"><div class="n">03</div><div class="t">이동 판단</div><div class="d">전방 카메라</div></div><div class="step"><div class="n">04</div><div class="t">현장 스캔</div><div class="d">하강 RGB-D</div></div><div class="step"><div class="n">05</div><div class="t">공간 선택</div><div class="d">요청점 대신 빈 곳</div></div><div class="step"><div class="n">06</div><div class="t">관람객 개입</div><div class="d">사람·장애물</div></div><div class="step"><div class="n">07</div><div class="t">결과</div><div class="d">착륙 또는 대기</div></div></div>
<div class="grid2" style="grid-template-columns:1.25fr .75fr">
  <div class="dash"><div><h3>지도 · 목적지</h3><p>물류 야드 / 상점 / 광장 / 학교</p><p>현재 위치와 실제 이동 궤적</p></div><div><h3>큰 RGB 카메라 + 판단 결과</h3><p>장애물 제외 · 안전 후보 · 요청 좌표 · 최종 선택점</p><p style="margin-top:66px;text-align:center;color:var(--blue);font-weight:900">“착륙 예정지에 사람이 진입하여 재탐색합니다.”</p></div><div><h3>상태 · 판단 이유</h3><p>이동 / 정지 / 우회 / 스캔 / 재탐색 / 착륙 / 보류</p><p>접촉 · 최소 여유 · 재선택 횟수</p></div><div class="controls"><span>배송 시작</span><span>사람 지나가기</span><span>장애물 추가</span><span>초기화</span></div></div>
  <div><div class="section">대표 시연</div><div class="box blue"><h3>1차 판단</h3><p>Foundry Yard의 지정 좌표에 놓인 화물을 피해 빈 바닥 선택</p></div><div class="box red" style="margin-top:9px"><h3>관람객 개입</h3><p>하강 중 사람이 착륙 예정지를 가로지름</p></div><div class="box green" style="margin-top:9px"><h3>기대 결과</h3><p>하강 중지 → 재탐색 → 다른 곳 선택 또는 안전 대기</p></div></div>
</div>"""))

    slides.append(slide(8, "앞으로 발전시킬 기술", "맵 확장보다 위험한 상황에서 내리지 않는 동작과 반복 검증을 우선", """
<div class="statement"><span class="bad">첫 번째 완료 조건:</span> 안전 후보가 없거나 센서가 오래되면 절대로 하강하지 않는다.</div>
<div class="road"><div class="box"><div class="k">PRIORITY 01</div><h3>후보 없음</h3><p>하강 금지<br>대기 · 재탐색 · 중단</p></div><div class="box"><div class="k">PRIORITY 02</div><h3>후보 안전성</h3><p>안전 영역 내부 확인<br>드론 크기만큼 여유 확보</p></div><div class="box"><div class="k">PRIORITY 03</div><h3>고도·회전</h3><p>실제 표면 높이와<br>드론 자세 반영</p></div><div class="box"><div class="k">PRIORITY 04</div><h3>이동 회피</h3><p>감속·정지·막힘 탈출<br>경로 복구</p></div><div class="box"><div class="k">PRIORITY 05</div><h3>객관적 평가</h3><p>접촉·관통·최소 여유<br>시간·보류·실패 기록</p></div></div>
<div class="section" style="margin-top:18px">통합 개발 순서</div>
<div class="flow"><div class="step"><div class="n">STAGE 1</div><div class="t">정적 착륙</div><div class="d">짧은 접근<br>후보 없음 금지</div></div><div class="step"><div class="n">STAGE 2</div><div class="t">동적 재평가</div><div class="d">사람 진입<br>정지·재선택</div></div><div class="step"><div class="n">STAGE 3</div><div class="t">이동 회피</div><div class="d">막힌 구간<br>정지·복구</div></div><div class="step"><div class="n">STAGE 4</div><div class="t">참여형 UI</div><div class="d">목적지 선택<br>반복 배송</div></div><div class="step"><div class="n">STAGE 5</div><div class="t">복층 확장</div><div class="d">캠퍼스·교량<br>산동네</div></div><div class="step"><div class="n">METRIC</div><div class="t">반복 평가</div><div class="d">성공·대기·실패<br>원인 기록</div></div><div class="step"><div class="n">RESULT</div><div class="t">통합 시연</div><div class="d">관람객 개입에<br>안전하게 반응</div></div></div>"""))

    slides.append(slide(9, "현재 결론과 다음 결정", "환경은 준비됐고, 이제 기술의 안전성과 통합 신뢰성을 증명할 단계", """
<div class="grid2">
  <div><div class="section">현재 확보한 것</div><div class="box green"><h3>시험 환경</h3><p>148동·22개 목적지·다층 지형을 포함한 배송 시험장</p></div><div class="box green" style="margin-top:10px"><h3>인지 프로토타입</h3><p>DINOv2와 거리 정보를 결합한 착륙 후보 생성·선택·판단 화면</p></div><div class="box green" style="margin-top:10px"><h3>기본 미션</h3><p>상태머신, 반응형 전방 회피, 하강 중 목표 재정렬, RViz 시각화</p></div></div>
  <div><div class="section">다음 회의에서 결정할 것</div><div class="box blue"><h3>첫 통합 시연 코스</h3><p>Foundry Yard의 짧은 접근과 정적 화물 회피부터 시작할 것인가?</p></div><div class="box blue" style="margin-top:10px"><h3>성공의 정의</h3><p>착륙만 성공으로 볼지, 안전 대기와 중단도 올바른 결과로 포함할 것인가?</p></div><div class="box blue" style="margin-top:10px"><h3>최종 UI 범위</h3><p>목적지 선택·사람 개입·결과 카드까지 어디를 졸업작품 범위로 확정할 것인가?</p></div></div>
</div>
<div class="statement" style="margin-top:20px"><span class="good">다양한 환경 + RGB-D 착륙 판단 + 기본 미션</span> → <span class="accent">안전 실패 처리 + 통합 검증 + 관람객 인터랙션</span></div>
"""))

    script = """<script>const s=[...document.querySelectorAll('.slide')];let i=0;function go(d){i=Math.max(0,Math.min(s.length-1,i+d));s[i].scrollIntoView({behavior:'smooth',block:'start'})}document.addEventListener('keydown',e=>{if(['ArrowRight','ArrowDown','PageDown',' '].includes(e.key)){e.preventDefault();go(1)}if(['ArrowLeft','ArrowUp','PageUp'].includes(e.key)){e.preventDefault();go(-1)}if(e.key==='Home'){i=0;go(0)}if(e.key==='End'){i=s.length-1;go(0)}})</script>"""
    document = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>서현은 · RGB-D 배송 드론 종합설계</title><style>{CSS}</style></head><body><div class="deck">{''.join(slides)}</div><div class="nav"><button onclick="go(-1)">◀ 이전</button><button onclick="go(1)">다음 ▶</button></div>{script}</body></html>"""
    output = OUT / "safe_landing_review_compact.html"
    output.write_text(document, encoding="utf-8")

    render_dir = OUT / "render_compact_html"
    render_dir.mkdir(exist_ok=True)
    override = "html,body{width:1600px!important;height:900px!important;overflow:hidden!important;background:#fff!important}.deck{padding:0!important;gap:0!important}.slide{margin:0!important}.nav{display:none!important}"
    for index, content in enumerate(slides, 1):
        page = f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><style>{CSS}{override}</style></head><body><div class='deck'>{content}</div></body></html>"
        (render_dir / f"slide_{index:02d}.html").write_text(page, encoding="utf-8")
    print(f"Created {output} with {len(slides)} slides; HTML-first; no video")
    return output


if __name__ == "__main__":
    build()
