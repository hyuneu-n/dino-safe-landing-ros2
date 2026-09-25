#!/usr/bin/env python3
"""H1/H2/H3 planner 로그를 표와 SVG 궤적으로 비교한다."""
import argparse
import json
import math
from pathlib import Path
import shutil


GOAL = (150.0, 0.0)


def esc(value):
    return str(value).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def read_run(folder):
    folder = Path(folder)
    summary = json.loads((folder / "summary.json").read_text())
    rows = [json.loads(line) for line in (folder / "planner.jsonl").read_text().splitlines()
            if line.strip()]
    positions = [r["position"][:2] for r in rows if r.get("status") == "OK" and r.get("position")]
    end = positions[-1] if positions else None
    summary["goal_distance_m"] = math.hypot(end[0] - GOAL[0], end[1] - GOAL[1]) if end else None
    summary["near_goal_10m"] = summary["goal_distance_m"] <= 10.0 if end else False
    return summary, positions


def make_svg(runs, path):
    width, height = 1200, 470
    margin, panel_w = 35, 360
    x_min, x_max, y_min, y_max = -160.0, 160.0, -180.0, 180.0

    def project(x, y, panel):
        left = margin + panel * (panel_w + 25)
        px = left + (x - x_min) / (x_max - x_min) * panel_w
        py = 430 - (y - y_min) / (y_max - y_min) * 360
        return px, py

    colors = ["#f59e0b", "#16a34a", "#2563eb"]
    chunks = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
              '<rect width="100%" height="100%" fill="#f8fafc"/>',
              '<style>text{font-family:DejaVu Sans,Arial,sans-serif;fill:#172033}.title{font-weight:700;font-size:18px}.small{font-size:12px}.axis{stroke:#cbd5e1;stroke-width:1}</style>']
    for panel, ((label, summary, positions), color) in enumerate(zip(runs, colors)):
        left = margin + panel * (panel_w + 25)
        chunks.append(f'<rect x="{left}" y="70" width="{panel_w}" height="360" fill="white" stroke="#cbd5e1"/>')
        # x/y zero axes
        x0, y0 = project(0, 0, panel)
        chunks.append(f'<line class="axis" x1="{left}" y1="{y0:.1f}" x2="{left+panel_w}" y2="{y0:.1f}"/>')
        chunks.append(f'<line class="axis" x1="{x0:.1f}" y1="70" x2="{x0:.1f}" y2="430"/>')
        # Explicit central blockers used by the test world (top view).
        for x, y, sx, sy in [(20,0,8,8),(20,16,9,9),(20,-16,11,11),
                              (35,11,8,8),(35,-11,8,8),(50,0,9,9),(50,14,10,10),
                              (50,-18,8,8),(65,12,8,8),(65,-12,8,8),(80,0,8,8),
                              (80,-14,10,10),(80,14,8,8)]:
            a, b = project(x-sx/2, y+sy/2, panel)
            c, d = project(x+sx/2, y-sy/2, panel)
            chunks.append(f'<rect x="{a:.1f}" y="{b:.1f}" width="{c-a:.1f}" height="{d-b:.1f}" fill="#334155" opacity=".72"/>')
        if positions:
            pts = " ".join(f"{project(x,y,panel)[0]:.1f},{project(x,y,panel)[1]:.1f}" for x,y in positions)
            chunks.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="3" stroke-linejoin="round"/>')
            sx, sy = project(*positions[0], panel)
            ex, ey = project(*positions[-1], panel)
            chunks.extend([f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="#111827"/>',
                           f'<circle cx="{ex:.1f}" cy="{ey:.1f}" r="6" fill="{color}" stroke="white" stroke-width="2"/>'])
        gx, gy = project(*GOAL, panel)
        chunks.append(f'<path d="M {gx-7:.1f} {gy:.1f} L {gx+7:.1f} {gy:.1f} M {gx:.1f} {gy-7:.1f} L {gx:.1f} {gy+7:.1f}" stroke="#dc2626" stroke-width="3"/>')
        chunks.append(f'<text class="title" x="{left}" y="28">{esc(label)}</text>')
        chunks.append(f'<text class="small" x="{left}" y="48">goal distance {summary["goal_distance_m"]:.1f}m · median {summary["plan_ms_median"]:.1f}ms · collisions {summary.get("trajectory_audit",{}).get("collision_sample_count","-")}</text>')
        chunks.append(f'<text class="small" x="{left+5}" y="448">x: -160…160m / y: -180…180m · red cross = goal</text>')
    chunks.append('</svg>')
    Path(path).write_text("\n".join(chunks))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--h1", required=True)
    ap.add_argument("--h2", required=True)
    ap.add_argument("--h3", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = []
    for horizon, folder in ((1,args.h1),(2,args.h2),(3,args.h3)):
        summary, positions = read_run(folder)
        runs.append((f"Horizon {horizon}", summary, positions))
        shutil.copyfile(Path(folder)/"planner.jsonl", out/f"h{horizon}_planner.jsonl")
        (out/f"h{horizon}_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2)+"\n")

    table = ["# Horizon 1·2·3 예비 비교", "",
             "동일 Gazebo 차단 회랑, 각 1회, 45초 실행 결과다. 반복 실험 전 예비 결과이며 최종 성능 주장이 아니다.", "",
             "| Horizon | 목표까지 남은 거리 | 목표 10m 이내 | 궤적 길이 | 계획시간 중앙값 | 재관측 | 관통 표본 |",
             "|---:|---:|:---:|---:|---:|---:|---:|"]
    for label, s, _ in runs:
        audit=s.get('trajectory_audit',{})
        table.append(f"| {s['horizon']} | {s['goal_distance_m']:.1f}m | {'예' if s['near_goal_10m'] else '아니오'} | "
                     f"{s['logged_route_length_m']:.1f}m | {s['plan_ms_median']:.1f}ms | "
                     f"{s['recovery_yaw_count']} | {audit.get('collision_sample_count','-')} |")
    table.extend(["", "현재 단일 실행에서는 Horizon 2만 세 차단 구간을 통과해 목표 10m 이내에 도달했다. "
                  "Horizon 1은 목표 방향을 크게 이탈했고 Horizon 3은 두 번째 차단 구간에서 안전 정지했다.", ""])
    (out/"comparison.md").write_text("\n".join(table))
    make_svg(runs, out/"horizon_trajectories.svg")
    print("\n".join(table))


if __name__ == "__main__":
    main()
