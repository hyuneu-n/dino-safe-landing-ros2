#!/usr/bin/env python3
"""ROS 2 web dashboard bridge for the Safe Landing public demo."""
import argparse
import io
import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
from PIL import Image as PILImage


ROOT = Path(__file__).resolve().parent


class SharedState:
    def __init__(self, manifest):
        self.lock = threading.Lock()
        self.started = time.time()
        self.mission = "CONNECTING"
        self.planning = {"status": "WAITING"}
        self.odom = None
        self.target = None
        self.selected_destination = None
        self.trajectory = []
        self.images = {"chase": None, "landing": None}
        self.destinations = manifest.get("landing_spots", [])

    def snapshot(self):
        with self.lock:
            return {
                "connected": self.odom is not None,
                "mission": self.mission,
                "planning": self.planning,
                "odom": self.odom,
                "target": self.target,
                "selected_destination": self.selected_destination,
                "trajectory": self.trajectory[-900:],
                "uptime": time.time() - self.started,
            }


def image_to_jpeg(msg):
    channels = max(1, msg.step // msg.width)
    arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    arr = arr[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    rgb = arr[:, :, :3]
    if msg.encoding.lower().startswith("bgr"):
        rgb = rgb[:, :, ::-1]
    out = io.BytesIO()
    PILImage.fromarray(np.ascontiguousarray(rgb)).save(out, "JPEG", quality=82)
    return out.getvalue()


def build_ros_node(shared):
    import rclpy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()

    class DashboardNode(Node):
        def __init__(self):
            super().__init__("safe_landing_dashboard")
            self.click_pub = self.create_publisher(PointStamped, "/clicked_point", 10)
            self.create_subscription(String, "/mission/status", self.on_mission, 10)
            self.create_subscription(String, "/planning/status", self.on_planning, 10)
            self.create_subscription(Odometry, "/drone/odom", self.on_odom,
                                     qos_profile_sensor_data)
            self.create_subscription(PointStamped, "/landing/target", self.on_target, 10)
            self.create_subscription(Image, "/drone/chase_cam/image_raw",
                                     lambda m: self.on_image("chase", m), qos_profile_sensor_data)
            self.create_subscription(Image, "/landing/overlay",
                                     lambda m: self.on_image("landing", m), qos_profile_sensor_data)

        def on_mission(self, msg):
            with shared.lock:
                shared.mission = msg.data

        def on_planning(self, msg):
            try:
                value = json.loads(msg.data)
            except (ValueError, TypeError):
                value = {"status": msg.data}
            with shared.lock:
                shared.planning = value

        def on_odom(self, msg):
            p = msg.pose.pose.position
            point = [round(p.x, 3), round(p.y, 3), round(p.z, 3)]
            with shared.lock:
                shared.odom = point
                if not shared.trajectory or sum((point[i] - shared.trajectory[-1][i]) ** 2
                                                for i in range(2)) > 1.0:
                    shared.trajectory.append(point)

        def on_target(self, msg):
            with shared.lock:
                shared.target = [msg.point.x, msg.point.y, msg.point.z]

        def on_image(self, name, msg):
            try:
                jpg = image_to_jpeg(msg)
            except Exception as exc:
                self.get_logger().warn(f"{name} image conversion failed: {exc}")
                return
            with shared.lock:
                shared.images[name] = jpg

        def select_destination(self, spot):
            msg = PointStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x, msg.point.y, msg.point.z = (
                float(spot["x"]), float(spot["y"]), float(spot.get("z", 0)))
            self.click_pub.publish(msg)
            with shared.lock:
                shared.selected_destination = {
                    "id": spot["id"], "label": spot["label"],
                    "x": spot["x"], "y": spot["y"], "z": spot.get("z", 0),
                    "validated": spot["id"] == "mcdonalds_delivery",
                }

    return rclpy, DashboardNode()


def make_handler(shared, node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def send_bytes(self, code, data, content_type):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/state":
                self.send_bytes(200, json.dumps(shared.snapshot(), ensure_ascii=False).encode(),
                                "application/json; charset=utf-8")
                return
            if path == "/api/destinations":
                self.send_bytes(200, json.dumps(shared.destinations, ensure_ascii=False).encode(),
                                "application/json; charset=utf-8")
                return
            if path in ("/api/chase.jpg", "/api/landing.jpg"):
                key = "chase" if "chase" in path else "landing"
                with shared.lock:
                    image = shared.images[key]
                if image is None:
                    self.send_bytes(503, b"camera not ready", "text/plain")
                else:
                    self.send_bytes(200, image, "image/jpeg")
                return
            rel = "index.html" if path == "/" else path.lstrip("/")
            file = (ROOT / rel).resolve()
            if ROOT not in file.parents or not file.is_file():
                self.send_error(404)
                return
            self.send_bytes(200, file.read_bytes(), mimetypes.guess_type(file.name)[0]
                            or "application/octet-stream")

        def do_POST(self):
            if urlparse(self.path).path != "/api/destination":
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
                spot = next(s for s in shared.destinations if s["id"] == body.get("id"))
                if spot.get("requires_elevation_support"):
                    raise ValueError("고도 목적지는 지면 높이 처리를 마친 뒤 활성화됩니다.")
                node.select_destination(spot)
                self.send_bytes(200, json.dumps({"ok": True, "destination": spot},
                                                ensure_ascii=False).encode(),
                                "application/json; charset=utf-8")
            except StopIteration:
                self.send_bytes(404, b'{"error":"unknown destination"}', "application/json")
            except (ValueError, json.JSONDecodeError) as exc:
                self.send_bytes(400, json.dumps({"error": str(exc)}, ensure_ascii=False).encode(),
                                "application/json; charset=utf-8")

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path,
                        default=ROOT.parent / "worlds/generated/metropolis_seed0.waypoints.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    shared = SharedState(manifest)
    rclpy, node = build_ros_node(shared)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(shared, node))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Dashboard: http://{args.host}:{args.port}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
