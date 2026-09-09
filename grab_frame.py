#!/usr/bin/env python3
"""grab_frame.py — /drone/down_cam/image_raw 첫 프레임을 PNG로 저장 (dino_seg 오프라인 테스트용)
사용: source /opt/ros/humble/setup.bash; python3 grab_frame.py [out.png] [topic]"""
import sys
import numpy as np
from PIL import Image as PImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else "frame.png"
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/drone/down_cam/image_raw"


class Grab(Node):
    def __init__(self):
        super().__init__("grab_frame")
        self.create_subscription(Image, TOPIC, self.cb, 5)
        self.done = False

    def cb(self, m):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        PImage.fromarray(a).save(OUT)
        self.get_logger().info(f"저장: {OUT}  ({m.width}x{m.height})")
        self.done = True


def main():
    rclpy.init()
    n = Grab()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=1.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
