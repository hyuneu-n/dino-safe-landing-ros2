#!/usr/bin/env python3
"""
run_comparison.py — 우리 방법 vs baseline 2개(PX4식 depth-only, OpenLander RGB-DNN) 3열 비교.
남은작업 #3 (baseline 2개 붙이기 + 3열 비교표).

절차 (seed마다 반복):
  1) worlds/generate_world.py로 seed 월드 생성 (없으면).
  2) gzserver 기동 → 드론을 목적지 상공(SCAN_ALT)으로 순간이동 → RGB+depth 한 장 캡처
     (mission_controller.py 전체 상태머신은 안 돌림 — SCAN 단계의 정적 스냅샷 하나면 충분).
  3) 정확히 같은 (rgb, depth, odom_xy, h_alt) 를 세 방법 모두에 입력해서 착륙점 선택.
  4) generate_layout()의 실제 3D 지오메트리로 계산한 GT 마스크로 각 선택의 안전 여부 +
     MOD(최근접 장애물까지 거리, m) 판정.
  5) N개 seed 집계 → results/comparison.md (표) + results/comparison.json (원자료).

비교가 공정하려면 지켜야 하는 것들 (gt_mask.py 상단에도 같은 얘기 있음):
  - "성공"의 정의 = 선택 지점의 드론 footprint 반경 안에 GT 장애물이 하나도 없음.
    표면 재질(잔디 등) 배제는 GT에 안 들어감 — 그건 우리 방법만의 부가 기능이라
    depth-only/RGB-DNN baseline과 비교하면 불공정해짐.
  - CONTEXT.md의 경고 그대로: 여기 나온 수치를 논문 mIoU와 직접 비교하지 말 것.
    같은 Gazebo 씬, 같은 GT, 같은 조건에서 셋을 돌린 상대 비교표일 뿐이다.

사용:
  source ~/venv_ros/bin/activate && source /opt/ros/humble/setup.bash
  cd ~/safe_landing/eval
  python3 run_comparison.py --seeds 5        # seed 0..4
  python3 run_comparison.py --seed-list 0,3,7
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SAFE_LANDING = os.path.dirname(HERE)
WORLDS_DIR = os.path.join(SAFE_LANDING, "worlds")
GENERATED_DIR = os.path.join(WORLDS_DIR, "generated")

for p in (WORLDS_DIR, SAFE_LANDING, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from generate_world import generate_layout, default_args, build_world_xml, validate_xml  # noqa: E402
from gt_mask import compute_gt_mask, min_obstacle_distance_m  # noqa: E402
import camera  # noqa: E402
from capture_frame import capture as capture_frame  # noqa: E402
from baseline_depth_only import pick_landing_point as depth_only_pick  # noqa: E402
from baseline_openlander import OpenLanderBaseline  # noqa: E402
from our_method import pick_landing_point as our_pick  # noqa: E402
from dino_seg import DinoSeg  # noqa: E402

SCAN_ALT = 12.0  # mission_controller.py의 SCAN_ALT와 일치
DRONE_FOOTPRINT_M = 1.0
GZ_BOOT_SEC = 12.0  # run_fixed.sh와 동일 관례 (world 로드 + 센서 초기화 대기)


def ensure_world(seed, layout_args, force=False):
    """target_jitter>0으로 부를 때는 seed별로 다시 만들어야 하므로(파일명이 jitter를
    안 담고 있음) force=True로 재생성한다 — 캐시된 jitter=0 세계를 잘못 재사용하는 걸 방지."""
    path = os.path.join(GENERATED_DIR, f"residential_delivery_seed{seed}.world")
    if force or not os.path.exists(path):
        os.makedirs(GENERATED_DIR, exist_ok=True)
        xml = build_world_xml(layout_args, seed)
        validate_xml(xml, f"seed{seed}")
        with open(path, "w", encoding="utf-8") as f:
            f.write(xml)
    return path


def start_gzserver(world_path, log_path):
    env = os.environ.copy()
    env.setdefault("GAZEBO_MODEL_DATABASE_URI", "http://127.0.0.1:9")  # 인터넷 fetch 방지 (run_fixed.sh 관례)
    env["GAZEBO_MODEL_PATH"] = os.path.expanduser("~/.gazebo/models") + ":" + env.get("GAZEBO_MODEL_PATH", "")
    log = open(log_path, "w")
    proc = subprocess.Popen(
        ["gzserver", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", world_path],
        stdout=log, stderr=subprocess.STDOUT, env=env, preexec_fn=os.setsid)
    return proc


def stop_gzserver(proc):
    # gzserver는 SIGTERM으로 안 죽는 경우를 실제로 겪어서 SIGKILL로 바로 감 (개발 중 확인됨)
    try:
        os.killpg(os.getpgid(proc.pid), 9)
    except Exception:
        pass
    subprocess.run(["fuser", "-k", "-9", "11345/tcp"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-x", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def meter_radius_to_px(h_alt, radius_m=DRONE_FOOTPRINT_M):
    m_per_px = (2.0 * h_alt * math.tan(1.396 / 2.0)) / camera.IMG_W
    return max(1, int(radius_m / max(m_per_px, 1e-3)))


def judge(gt_mask, u, v, h_alt, odom_xy):
    """u,v 중심 DRONE_FOOTPRINT_M 반경 안에 GT 장애물이 하나라도 있으면 실패로 판정."""
    if u is None:
        return False, None
    r = meter_radius_to_px(h_alt)
    H, W = gt_mask.shape
    u0, u1 = max(0, int(u - r)), min(W, int(u + r) + 1)
    v0, v1 = max(0, int(v - r)), min(H, int(v + r) + 1)
    patch = gt_mask[v0:v1, u0:u1]
    success = not bool(patch.any())
    mod = min_obstacle_distance_m(gt_mask, u, v, h_alt, odom_xy)
    return success, mod


def run_one_seed(seed, ol, dino_seg, results_dir, layout_args):
    print(f"\n=== seed {seed} ===", flush=True)
    world_path = ensure_world(seed, layout_args, force=(layout_args.target_jitter > 0))
    layout = generate_layout(layout_args, seed)
    target_xy = layout["target_xy"]

    log_path = os.path.join(results_dir, f"gz_seed{seed}.log")
    proc = start_gzserver(world_path, log_path)
    try:
        time.sleep(GZ_BOOT_SEC)
        rgb, depth = capture_frame(target_xy[0], target_xy[1], SCAN_ALT, settle_sec=4.0)
    finally:
        stop_gzserver(proc)

    if rgb is None or depth is None:
        print(f"[seed {seed}] 캡처 실패 — 건너뜀 (gz 로그: {log_path})", flush=True)
        return None

    Image.fromarray(rgb).save(os.path.join(results_dir, f"seed{seed}_rgb.png"))
    np.save(os.path.join(results_dir, f"seed{seed}_depth.npy"), depth)

    gt = compute_gt_mask(layout, odom_xy=target_xy, h_alt=SCAN_ALT)
    row = {"seed": seed}

    def add(name, pick_result):
        if pick_result:
            u, v = pick_result
            succ, mod = judge(gt, u, v, SCAN_ALT, target_xy)
            row[name] = {"picked": True, "success": bool(succ), "mod_m": mod, "u": float(u), "v": float(v)}
        else:
            row[name] = {"picked": False, "success": False, "mod_m": None}

    r = our_pick(dino_seg, rgb, depth, odom_xy=target_xy, h_alt=SCAN_ALT, target_xy=target_xy)
    add("ours", (r[1], r[2]) if r else None)

    r = depth_only_pick(depth, target_xy, SCAN_ALT, target_xy)
    add("depth_only", (r[1], r[2]) if r else None)

    r = ol.pick_landing_point(rgb, target_xy, SCAN_ALT, target_xy)
    add("openlander", (r[1], r[2]) if r else None)

    print(f"[seed {seed}] ours={row['ours']} | depth_only={row['depth_only']} | "
          f"openlander={row['openlander']}", flush=True)
    return row


METHOD_LABELS = {
    "ours": "우리 방법 (DINOv2 + depth)",
    "depth_only": "PX4식 depth-only",
    "openlander": "OpenLander (RGB-only DNN)",
}


def summarize(rows):
    n = len(rows)
    summary = {}
    lines = ["| 방법 | 착륙점 제시율 | 성공률(장애물 회피) | 평균 MOD(m) |", "|---|---|---|---|"]
    for m, label in METHOD_LABELS.items():
        picked = [r[m] for r in rows if r[m]["picked"]]
        succ = [r[m] for r in rows if r[m]["success"]]
        mods = [r[m]["mod_m"] for r in rows if r[m]["mod_m"] is not None]
        picked_rate = len(picked) / n * 100 if n else 0.0
        succ_rate = len(succ) / n * 100 if n else 0.0
        mean_mod = float(np.mean(mods)) if mods else None
        summary[m] = {"n": n, "picked": len(picked), "success": len(succ),
                       "picked_rate": picked_rate, "success_rate": succ_rate, "mean_mod_m": mean_mod}
        mod_str = f"{mean_mod:.2f}" if mean_mod is not None else "N/A"
        lines.append(f"| {label} | {picked_rate:.0f}% ({len(picked)}/{n}) | "
                     f"{succ_rate:.0f}% ({len(succ)}/{n}) | {mod_str} |")
    return "\n".join(lines), summary


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=5, help="seed 0..N-1 (기본 5)")
    ap.add_argument("--seed-list", default=None, help="쉼표구분 seed 목록 (예: 0,3,7). 지정 시 --seeds 무시")
    ap.add_argument("--outdir", default=os.path.join(HERE, "results"))
    ap.add_argument("--target-jitter", type=float, default=4.0,
                     help="목적지 채점 시나리오 위치 지터(m). 기본 4.0 — 0이면 모든 seed가 "
                          "완전히 똑같은 목적지 장면을 보게 돼(넓은 배경 건물은 안 보이는 좁은 "
                          "하강뷰라 회랑 seed 차이가 전혀 반영 안 됨) 비교가 무의미해짐")
    args = ap.parse_args()

    seeds = [int(s) for s in args.seed_list.split(",")] if args.seed_list else list(range(args.seeds))
    os.makedirs(args.outdir, exist_ok=True)
    layout_args = default_args(target_jitter=args.target_jitter)

    # 정리 (혹시 이전 실행이 죽어서 안 지워진 gzserver)
    subprocess.run(["pkill", "-9", "-x", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["fuser", "-k", "-9", "11345/tcp"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

    print("[run_comparison] OpenLander 모델 로드 중...", flush=True)
    ol = OpenLanderBaseline()
    print("[run_comparison] DINOv2 로드 중...", flush=True)
    dino_seg = DinoSeg(prefer="dinov2")

    rows = []
    for seed in seeds:
        row = run_one_seed(seed, ol, dino_seg, args.outdir, layout_args)
        if row:
            rows.append(row)

    if not rows:
        raise SystemExit("모든 seed 캡처 실패 — 비교표를 만들 수 없음")

    table, summary = summarize(rows)
    print("\n" + table)

    with open(os.path.join(args.outdir, "comparison.json"), "w", encoding="utf-8") as f:
        json.dump({"rows": rows, "summary": summary, "scan_alt": SCAN_ALT,
                   "footprint_m": DRONE_FOOTPRINT_M}, f, ensure_ascii=False, indent=2)
    with open(os.path.join(args.outdir, "comparison.md"), "w", encoding="utf-8") as f:
        f.write("# 착륙지 선정 3열 비교 (같은 Gazebo 씬, 같은 GT, seed=" +
                ",".join(str(r["seed"]) for r in rows) + ")\n\n")
        f.write(table + "\n\n")
        f.write("주의: 이 수치는 우리 절차생성 Gazebo 씬 안에서의 상대 비교이지, "
                "논문에 보고된 실사 데이터셋 mIoU와 직접 비교할 수 없음 (CONTEXT.md 참고).\n")
    print(f"\n[ok] results -> {args.outdir}/comparison.md , comparison.json")


if __name__ == "__main__":
    main()
