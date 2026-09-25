# 종합설계 발표 자료

## 현재 검토본 — Compact HTML

사용자가 제공한 이전 발표 `종설_가장 마지막 진행상황.pptx`의 정보 밀도와 구성에 맞춰 만든 **10장 HTML 우선 편집본**이다.

- [Compact HTML](safe_landing_review_compact.html)
- [Compact PowerPoint](safe_landing_review_compact.pptx): 10장, 발표자 노트 내장
- [Compact 발표 대본](speaker_notes_compact.md): 약 8~10분
- [10장 전체 미리보기](overview_compact.jpg)
- 생성 코드: `build_html_deck_compact.py`, `build_pptx_compact.py`

표지는 `종합설계 / 서현은`으로 만들었다. 본문에는 반복되는 소속·이름 머리말을 두지 않았고 Part 1–5 전환 슬라이드와 환경 영상도 없다. PPTX는 확정된 HTML 렌더 화면을 사용한 레이아웃 고정형이며, 각 슬라이드의 발표자 노트에 대본을 넣었다.

## 이전 V2

새 지도교수님이 프로젝트를 처음 듣는 상황을 기준으로 다시 구성한 발표다. 중심은 맵 자체가 아니라 **문제 정의 → 기술과 구현 수준 → 시험 환경 → 최종 인터랙션 → 기술 발전 계획**이다. 사용자 제공 템플릿 `36_발표_시간축과confound.html`의 CSS와 화면 구성을 사용했다.

## 바로 사용할 파일

- [PowerPoint](safe_landing_first_review_v2.pptx): 23장, 화면 고정형
- [HTML 원본](safe_landing_first_review_v2.html): 키보드 방향키·Page Up/Down으로 진행
- [PDF](safe_landing_first_review_v2.pdf): 23페이지
- [전체 슬라이드 한눈에 보기](overview_v2.jpg)
- [발표 구성안](PRESENTATION_PLAN_V2.md): 주장 범위와 슬라이드별 근거
- [발표자 노트](speaker_notes_v2.md): 약 8~10분 기준

PPTX는 템플릿 화면이 PowerPoint에서 흐트러지지 않도록 렌더링 이미지 한 장을 슬라이드 한 장에 배치했다. 문구나 레이아웃은 `build_html_deck_v2.py` 또는 생성된 HTML에서 수정한 뒤 다시 렌더링한다.

## 발표 흐름

| 구간 | 내용 | 실제 슬라이드 |
|---|---|---:|
| 도입 | 좌표 도달과 안전 착륙의 차이 | 1–2 |
| Part 1 | 만들고자 하는 시스템과 범위 | 3–5 |
| Part 2 | 스택, 구조, 알고리즘, 제어, 구현·평가 상태 | 6–12 |
| Part 3 | 현재 맵 캡처와 목적지 조건 | 13–15 |
| Part 4 | 관람객 경험, 화면, 대표 시나리오 | 16–19 |
| Part 5 | 기술 우선순위와 개발 순서 | 20–22 |
| 결론 | 확보한 것과 다음 의사결정 | 23 |

맵은 내용 슬라이드 2장만 사용한다. 환경 투어 영상과 영상 링크는 포함하지 않았다.

## 발표 전에 바꿀 것

1. 표지의 `[이름 입력]`을 발표자 이름 또는 팀명으로 바꾼다.
2. 발표 시간이 5분이면 Part 구분 슬라이드는 빠르게 넘기고, 기술 세부와 기존 평가 설명을 줄인다.
3. HTML을 수정했다면 아래 순서로 23개 PNG, PPTX, PDF를 다시 만든다.
4. 최종 파일을 실제 발표 PC에서 전체 화면으로 한 번 확인한다.

## 기술 표현의 경계

- 위치는 Gazebo odom을 사용한다. 완전한 비전 자율비행이라고 표현하지 않는다.
- 22/24는 이전 단일 회랑의 정지 장면 착륙점 선택 결과다. 새 도시 배송 성공률이 아니다.
- DINO 군집은 사람이나 자동차의 이름을 판별하는 객체 검출이 아니다.
- 높이 있는 목적지 4곳과 도시 전체 코스는 아직 자동 비행 검증이 끝나지 않았다.
- 대시보드와 관람객 버튼은 최종 결과물 설계이며 현재 구현 완료 기능이 아니다.

## 재생성

```bash
python3 docs/presentation/build_html_deck_v2.py

# render_v2_html/slide_*.html을 1800×1080 PNG로 렌더링한 뒤
PYTHONPATH=/tmp/safe_landing_presentation_deps \
  python3 docs/presentation/build_pptx_v2.py
```

이번 산출물의 23개 렌더링 PNG는 `render_v2_png/`에 있다. `build_pptx_v2.py`는 여기에서 PPTX와 PDF를 함께 만든다. 패키지가 없으면 임시 경로에 설치한다.

```bash
python3 -m pip install --target /tmp/safe_landing_presentation_deps python-pptx
```

기존 `safe_landing_first_review.*`, `speaker_notes.md`, `overview.jpg`는 이전 16장 버전이다. V2 발표에는 사용하지 않는다.
