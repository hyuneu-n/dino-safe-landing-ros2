#!/usr/bin/env python3
"""
capture_frame.py — 실행 중인 gzserver에서 드론을 지정한 pose로 순간이동시키고
down_cam RGB + down_depth를 한 장 캡처. mission_controller.py의 전체 상태머신을 돌리지
않고, run_comparison.py가 "목적지 상공에서 스캔 중"인 정적 스냅샷 하나만 빠르게 얻으려고
쓰는 유틸리티. standalone 실행도 가능:

  source ~/venv_ros/bin/activate   # DINO는 안 쓰지만 rclpy/PIL 등은 여기 있음
  python3 capture_frame.py --x 150 --y 0 --z 12 --out /tmp/frame

--cam으로 down_cam 대신 chase_cam(비스듬한 뒤쪽 각도)을 찍을 수 있음 — 메시가 실제로
똑바로 서 있는지(방향) 확인은 down_cam(수직으로 아래만 봄)으로는 안 되고 이게 필요함.
"""
import argparse
import math
import time
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from gazebo_msgs.srv import SetEntityState

DRONE_NAME = "delivery_drone"
CAM_TOPICS = {
    "down": "/drone/down_cam/image_raw",
    "chase": "/drone/chase_cam/image_raw",
    "iso": "/drone/iso_cam/image_raw",
}


def img_to_np_rgb(msg):
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, -1)
    return a[:, :, :3].copy()


def img_to_np_depth(msg):
    return np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.width).copy()


class FrameCapture(Node):
    def __init__(self, x, y, z, settle_sec, yaw=0.0, cam="down"):
        super().__init__("frame_capture")
        self.x, self.y, self.z, self.settle_sec, self.yaw = x, y, z, settle_sec, yaw
        self.cam = cam
        self.need_depth = (cam == "down")
        self.rgb = self.depth = None
        self.done = False
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.create_subscription(RosImage, CAM_TOPICS[cam], self.on_rgb, 5)
        if self.need_depth:
            self.create_subscription(RosImage, "/drone/down_depth/depth/image_raw", self.on_depth, 5)
        if not self.cli.wait_for_service(timeout_sec=20.0):
            self.destroy_node()
            raise RuntimeError("Gazebo set_entity_state service unavailable after 20 seconds")
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.2, self.tick)

    def _teleport(self):
        req = SetEntityState.Request()
        req.state.name = DRONE_NAME
        req.state.pose.position.x = float(self.x)
        req.state.pose.position.y = float(self.y)
        req.state.pose.position.z = float(self.z)
        req.state.pose.orientation.z = math.sin(self.yaw / 2.0)
        req.state.pose.orientation.w = math.cos(self.yaw / 2.0)
        req.state.reference_frame = "world"
        self.cli.call_async(req)

    def on_rgb(self, m): self.rgb = img_to_np_rgb(m)
    def on_depth(self, m): self.depth = img_to_np_depth(m)

    def tick(self):
        self._teleport()  # kinematic이라 드리프트는 안 하지만, 매 틱 다시 박아서 확실히 함
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        ready = self.rgb is not None and (self.depth is not None or not self.need_depth)
        if elapsed >= self.settle_sec and ready:
            self.done = True


def capture(x, y, z, settle_sec=3.0, timeout_sec=30.0, yaw=0.0, cam="down"):
    """반환: (rgb HxWx3 uint8, depth HxW float32 또는 None). 못 받으면 (None, None).
    cam="chase"/"iso"는 depth가 없음(down_depth는 down_cam 전용) — depth는 None으로 옴.

    rclpy.spin(node)를 안 쓰는 이유: tick() 콜백 안에서 rclpy.shutdown()을 불러
    spin()을 멈추게 하는 흔한 패턴이 이 환경(rclpy 버전?)에서는 spin()이 절대
    안 돌아오는 걸로 확인됨 (t=4.0s 로그까지 찍히고 그대로 행) — 그래서 직접
    spin_once() 루프를 돌려 done 플래그로 빠져나온다."""
    rclpy.init()
    node = FrameCapture(x, y, z, settle_sec, yaw=yaw, cam=cam)
    start = time.time()
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.3)
        if time.time() - start > timeout_sec:
            node.get_logger().warn(f"{timeout_sec}s 안에 프레임을 못 받음 — 타임아웃")
            break
    rgb, depth = node.rgb, node.depth
    node.destroy_node()
    rclpy.shutdown()
    return rgb, depth


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--z", type=float, required=True)
    ap.add_argument("--out", required=True, help="출력 접두어 (…_rgb.png, …_depth.npy)")
    ap.add_argument("--settle", type=float, default=3.0, help="순간이동 후 안정화 대기(초)")
    ap.add_argument("--yaw", type=float, default=0.0, help="드론 yaw(rad) — chase_cam 방향 조절용")
    ap.add_argument("--cam", choices=list(CAM_TOPICS), default="down",
                     help="down(수직 아래, 기본) / chase(비스듬한 뒤쪽) / iso(대각선 3인칭)")
    args = ap.parse_args()
    rgb, depth = capture(args.x, args.y, args.z, args.settle, yaw=args.yaw, cam=args.cam)
    if rgb is None:
        raise SystemExit("캡처 실패 — gzserver가 안 떠 있거나 토픽이 안 들어옴")
    Image.fromarray(rgb).save(args.out + "_rgb.png")
    if depth is not None:
        np.save(args.out + "_depth.npy", depth)
    print(f"[ok] {args.out}_rgb.png" + (f" / {args.out}_depth.npy" if depth is not None else ""))


if __name__ == "__main__":
    main()
