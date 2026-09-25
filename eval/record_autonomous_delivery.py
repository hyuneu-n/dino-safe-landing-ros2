#!/usr/bin/env python3
"""Encode the real drone chase-camera stream until the autonomous mission lands."""
import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont, ImageOps


def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--destination", default="McDonald's Delivery")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--tail", type=float, default=3)
    args = parser.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()
    node = Node("autonomous_delivery_recorder")
    latest = {"image": None, "mission": "WAITING", "planning": {}, "odom": None}

    def on_image(msg):
        channels = max(1, msg.step // msg.width)
        arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
        arr = arr[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        rgb = arr[:, :, :3].copy()
        if msg.encoding.lower().startswith("bgr"):
            rgb = rgb[:, :, ::-1]
        latest["image"] = rgb

    def on_mission(msg):
        latest["mission"] = msg.data

    def on_planning(msg):
        try:
            latest["planning"] = json.loads(msg.data)
        except (ValueError, TypeError):
            latest["planning"] = {"status": msg.data}

    def on_odom(msg):
        p = msg.pose.pose.position
        latest["odom"] = (p.x, p.y, p.z)

    node.create_subscription(Image, "/drone/chase_cam/image_raw", on_image,
                             qos_profile_sensor_data)
    node.create_subscription(String, "/mission/status", on_mission, 10)
    node.create_subscription(String, "/planning/status", on_planning, 10)
    node.create_subscription(Odometry, "/drone/odom", on_odom, qos_profile_sensor_data)

    deadline = time.monotonic() + 60
    while latest["image"] is None or latest["odom"] is None:
        if time.monotonic() > deadline:
            raise TimeoutError("chase camera/odometry did not become ready")
        rclpy.spin_once(node, timeout_sec=.1)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = subprocess.Popen([
        "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
        "-pix_fmt", "rgb24", "-s", "1280x720", "-r", str(args.fps), "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "19",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.out),
    ], stdin=subprocess.PIPE)
    title_font, main_font, small_font = font(24, True), font(18, True), font(15)
    start = time.monotonic()
    next_frame = start
    landed_at = None
    frames = 0
    status_counts = {}
    completed = False
    try:
        while time.monotonic() - start < args.timeout:
            while time.monotonic() < next_frame:
                rclpy.spin_once(node, timeout_sec=min(.03, next_frame - time.monotonic()))
            rclpy.spin_once(node, timeout_sec=0)
            elapsed = time.monotonic() - start
            mission = latest["mission"]
            phase = mission.split()[0] if mission else "WAITING"
            status_counts[phase] = status_counts.get(phase, 0) + 1
            if phase == "LANDED":
                landed_at = landed_at or time.monotonic()
                if time.monotonic() - landed_at >= args.tail:
                    completed = True

            frame = PILImage.fromarray(latest["image"])
            frame = ImageOps.fit(frame, (1280, 720), method=PILImage.Resampling.LANCZOS)
            draw = ImageDraw.Draw(frame, "RGBA")
            draw.rounded_rectangle((22, 18, 596, 104), radius=12, fill=(8, 15, 23, 205))
            draw.text((42, 30), "AUTONOMOUS DELIVERY", font=title_font,
                      fill=(245, 199, 87, 255))
            draw.text((42, 67), f"Destination  |  {args.destination}", font=small_font,
                      fill=(234, 239, 243, 255))
            draw.rounded_rectangle((22, 625, 1258, 702), radius=12, fill=(8, 15, 23, 210))
            odom = latest["odom"] or (0, 0, 0)
            plan = latest["planning"]
            plan_status = plan.get("status", "WAITING") if isinstance(plan, dict) else "WAITING"
            cost = plan.get("selected_cost", plan.get("total_cost")) if isinstance(plan, dict) else None
            cost_text = f"  |  Local cost {cost:.2f}" if isinstance(cost, (int, float)) else ""
            draw.text((42, 640), f"MISSION  {phase}", font=main_font,
                      fill=(103, 238, 154, 255) if phase != "LANDED" else (245, 199, 87, 255))
            draw.text((42, 672),
                      f"CHASE CAMERA  |  x {odom[0]:+.1f}  y {odom[1]:+.1f}  alt {odom[2]:.1f} m"
                      f"  |  Planner {plan_status}{cost_text}",
                      font=small_font, fill=(221, 229, 235, 255))
            draw.text((1120, 36), f"{elapsed:05.1f}s", font=main_font, fill=(255, 255, 255, 240))
            ffmpeg.stdin.write(frame.tobytes())
            frames += 1
            if frames % (args.fps * 10) == 0:
                frame.save(args.out.parent / f"preview_{frames:05d}.jpg", quality=90)
                print(f"[record] {elapsed:5.1f}s {phase} pos=({odom[0]:.1f},{odom[1]:.1f},{odom[2]:.1f})",
                      flush=True)
            if completed:
                break
            next_frame += 1.0 / args.fps
        ffmpeg.stdin.close()
        return_code = ffmpeg.wait(timeout=60)
        if return_code:
            raise RuntimeError(f"ffmpeg exited with {return_code}")
    finally:
        if ffmpeg.poll() is None:
            ffmpeg.terminate()
            ffmpeg.wait(timeout=15)
        metadata = {
            "kind": "autonomous_delivery_chase_camera",
            "autonomous_flight": True,
            "destination": args.destination,
            "completed": completed,
            "duration_seconds": frames / args.fps,
            "fps": args.fps,
            "frames": frames,
            "final_mission_status": latest["mission"],
            "final_odom": latest["odom"],
            "phase_frame_counts": status_counts,
        }
        args.out.with_suffix(".json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    print(f"[{'ok' if completed else 'timeout'}] {args.out} ({frames / args.fps:.1f}s)", flush=True)
    if not completed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
