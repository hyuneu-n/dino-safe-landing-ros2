#!/usr/bin/env python3
"""mission_controller의 planner JSONL을 재현 가능한 요약으로 변환한다."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics

try:
    from .trajectory_clearance import audit_trajectory
except ImportError:  # 스크립트로 직접 실행할 때
    from trajectory_clearance import audit_trajectory


def percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    return s[round((len(s) - 1) * q)]


def summarize(rows, world_path=None):
    ok = [r for r in rows if r.get("status") == "OK"]
    positions = [r.get("position") for r in ok if r.get("position")]
    route_length = 0.0
    for a, b in zip(positions[:-1], positions[1:]):
        route_length += math.hypot(b[0] - a[0], b[1] - a[1])
    plan_ms = [float(r["plan_ms"]) for r in ok if r.get("plan_ms") is not None]
    clearance = [float(r["min_clearance_m"]) for r in ok
                 if r.get("min_clearance_m") is not None]
    turning = [r for r in ok if any(abs(float(v)) > 1e-6
                                    for v in (r.get("selected_steering_deg") or []))]
    start = positions[0] if positions else None
    end = positions[-1] if positions else None
    progress = None
    if start and end:
        progress = math.hypot(end[0] - start[0], end[1] - start[1])
    result = {
        "record_count": len(rows),
        "status_counts": dict(Counter(r.get("status", "UNKNOWN") for r in rows)),
        "horizon": ok[0].get("horizon") if ok else None,
        "turning_plan_count": len(turning),
        "turning_fraction": len(turning) / len(ok) if ok else None,
        "recovery_yaw_count": sum(r.get("status") == "RECOVERY_YAW" for r in rows),
        "plan_ms_median": statistics.median(plan_ms) if plan_ms else None,
        "plan_ms_p95": percentile(plan_ms, 0.95),
        "plan_ms_max": max(plan_ms) if plan_ms else None,
        "minimum_observed_clearance_m": min(clearance) if clearance else None,
        "logged_route_length_m": route_length,
        "net_progress_m": progress,
        "start_position": start,
        "end_position": end,
        "selected_steering_sequences": {
            str(k): v for k, v in Counter(
                tuple(r.get("selected_steering_deg") or []) for r in ok).items()
        },
    }
    if world_path:
        result["trajectory_audit"] = audit_trajectory(positions, world_path)
    return result


def markdown(summary):
    def f(value, digits=3):
        return "-" if value is None else f"{value:.{digits}f}"
    lines = [
        "# 지역 경로 계획 실행 요약",
        "",
        f"- Horizon: {summary['horizon']}",
        f"- 기록 수: {summary['record_count']}",
        f"- 상태: `{json.dumps(summary['status_counts'], ensure_ascii=False)}`",
        f"- 방향 전환 계획: {summary['turning_plan_count']}회 "
        f"({f(100 * summary['turning_fraction'], 1) if summary['turning_fraction'] is not None else '-'}%)",
        f"- 경로 없음 재관측 회전: {summary['recovery_yaw_count']}회",
        f"- 계획 시간 median / p95 / max: {f(summary['plan_ms_median'])} / "
        f"{f(summary['plan_ms_p95'])} / {f(summary['plan_ms_max'])} ms",
        f"- 관측 최소 여유거리: {f(summary['minimum_observed_clearance_m'])} m",
        f"- 로그 궤적 길이: {f(summary['logged_route_length_m'])} m",
        f"- 순변위: {f(summary['net_progress_m'])} m",
        f"- 시작: `{summary['start_position']}`",
        f"- 종료: `{summary['end_position']}`",
    ]
    audit = summary.get("trajectory_audit")
    if audit:
        lines.extend([
            f"- 궤적-충돌체 최소 표면 여유: {f(audit['minimum_surface_clearance_m'])} m",
            f"- 최근접 충돌체: `{audit['nearest_obstacle']}`",
            f"- 기체 반경 {audit['drone_radius_m']:.2f}m 기준 관통 표본: "
            f"{audit['collision_sample_count']}개",
        ])
    lines.extend([
        "",
        "`minimum_observed_clearance_m`는 계획 당시 Depth 표면과 후보 경로 사이 거리다. "
        "월드가 주어졌을 때의 궤적 검사는 SDF에 명시된 충돌체만 포함하며 model:// include의 "
        "내부 충돌체는 포함하지 않는다.",
        "",
    ])
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--json")
    ap.add_argument("--markdown")
    ap.add_argument("--world")
    args = ap.parse_args()
    rows = [json.loads(line) for line in Path(args.log).read_text().splitlines() if line.strip()]
    result = summarize(rows, args.world)
    text = markdown(result)
    print(text)
    if args.json:
        Path(args.json).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    if args.markdown:
        Path(args.markdown).write_text(text)


if __name__ == "__main__":
    main()
