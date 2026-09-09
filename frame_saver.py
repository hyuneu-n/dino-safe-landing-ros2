#!/usr/bin/env python3
"""
frame_saver.py — RViz 패널·카메라 *개별 녹화*용 노드

여러 image 토픽을 구독해서 각자 번호매김 PNG로 $FRAMES_DIR/<prefix>_NNNN.png 저장.
런 끝나면 ffmpeg로 prefix별 MP4 합치면 됨:
    ffmpeg -framerate 5 -i overlay_%04d.png -c:v libx264 -pix_fmt yuv420p overlay.mp4

환경변수:
    FRAMES_DIR   저장 디렉토리 (run_demo.sh가 자동 세팅)
"""
import os
import time
import numpy as np
from PIL import Image as PImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

FRAMES_DIR = os.environ.get("FRAMES_DIR", os.path.expanduser("~/safe_landing/frames/default"))
os.makedirs(FRAMES_DIR, exist_ok=True)

# (topic, prefix, target_hz) — 부드러운 영상 위해 카메라 15Hz, overlay는 1Hz 소스라 그대로
STREAMS = [
    ("/drone/down_cam/image_raw",        "downcam",   15),
    ("/drone/chase_cam/image_raw",       "chasecam",  15),
    ("/drone/iso_cam/image_raw",         "isocam",    15),
    ("/drone/front_depth/image_raw",     "frontview", 10),   # RGB(depth카메라의 가시)
    ("/drone/front_depth/depth/image_raw","frontdepth",10),  # 32FC1 — 회색 변환 저장
    ("/landing/overlay",                  "overlay",    2),  # detector 1Hz → 2Hz 샘플 OK
    ("/landing/dino_pca",                 "pca",        2),
]


class FrameSaver(Node):
    def __init__(self):
        super().__init__("frame_saver")
        self.state = {}
        for topic, prefix, hz in STREAMS:
            self.state[prefix] = {"count": 0, "last_t": 0.0, "min_dt": 1.0 / hz}
            self.create_subscription(
                Image, topic, lambda msg, p=prefix: self.on_image(p, msg), 1)
        self.get_logger().info(f"frame_saver 시작 — 저장 폴더: {FRAMES_DIR}")
        self.get_logger().info(f"감시 토픽: {[s[0] for s in STREAMS]}")

    def on_image(self, prefix, msg):
        s = self.state[prefix]
        now = time.time()
        if now - s["last_t"] < s["min_dt"]:
            return
        s["last_t"] = now
        arr = self._to_rgb(msg)
        if arr is None:
            return
        n = s["count"]
        try:
            PImage.fromarray(arr).save(os.path.join(FRAMES_DIR, f"{prefix}_{n:04d}.png"))
            s["count"] = n + 1
            if n % 30 == 0 and n > 0:
                self.get_logger().info(f"{prefix}: {n} frames")
        except Exception as e:
            self.get_logger().warn(f"{prefix} save fail: {e}")

    @staticmethod
    def _to_rgb(msg):
        h, w, enc = msg.height, msg.width, msg.encoding
        if enc in ("rgb8", "bgr8"):
            a = np.frombuffer(bytes(msg.data), np.uint8).reshape(h, w, 3)
            return a[:, :, ::-1] if enc == "bgr8" else a
        if enc == "32FC1":   # depth
            d = np.frombuffer(bytes(msg.data), np.float32).reshape(h, w)
            d = np.where(np.isfinite(d), d, 200.0)
            # 가까울수록 흰색, 멀수록 검은색 (드론 관점 — 가까운 게 위험)
            vis = np.clip(255 - d * 1.5, 0, 255).astype(np.uint8)
            return np.stack([vis] * 3, -1)
        return None


def main():
    rclpy.init()
    n = FrameSaver()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
