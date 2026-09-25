#!/usr/bin/env python3
"""
mission_controller.py — 배송 드론 미션 상태머신 (Gazebo Classic + ROS2)

드론을 /gazebo/set_entity_state 서비스로 kinematic 제어한다.
  IDLE → TAKEOFF → CRUISE(고층빌딩 사이 통과) → ARRIVE → SCAN(착륙지 탐지 대기)
       → DESCEND(탐지된 지점으로 하강) → LANDED

구독:
  /landing/target   geometry_msgs/PointStamped   (착륙지 탐지 노드가 발행한 안전 착륙점, world frame)
발행:
  /mission/status          std_msgs/String              (현재 상태)
  /mission/setpoint        geometry_msgs/PoseStamped    (드론 목표 위치, 디버그용)
  /planning/status         std_msgs/String              (후보 수, 비용 성분, 처리 시간)
  /planning/local_costmap  nav_msgs/OccupancyGrid       (depth 기반 지역 비용 지도)
  /planning/candidates     visualization_msgs/MarkerArray (전체 후보/탈락/선택 경로)
  /planning/selected_path  nav_msgs/Path                (선택된 짧은 경로)

실행:
  source /opt/ros/humble/setup.bash
  python3 mission_controller.py
"""
import json
import math
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from local_path_planner import (
    PlannerConfig, local_to_world, plan_local_path, world_to_local,
)

DRONE_NAME = "delivery_drone"
RATE_HZ = 30.0
CRUISE_ALT = 22.0          # 고층빌딩(56~80m) 아래 협곡 통과 고도
SCAN_ALT = 12.0            # 착륙지 스캔 호버 고도
SPEED_CRUISE = 8.0         # m/s (330m 경로라 좀 빠르게)
SPEED_DESCEND = 0.6        # m/s
SCAN_HOLD_SEC = 8.0        # 스캔 호버 유지 시간 (탐지 안정화)
START_XY = (-150.0, 0.0)   # 외곽 출발지 (Fuel 메시 마을 시작점)
TARGET_XY = (150.0, 0.0)   # GPS로 받은 거친 배송 목적지 (기본값 — self.target_xy 초기값)
# 탐지 실패 시 폴백 좌표는 (self.target_xy[0], self.target_xy[1], 0.30) — SCAN 상태에서 동적 계산

# ── 하강 중 동적 재평가 (보행자 진입 등) ──
COMMIT_ALT = 0.7           # 이 고도 아래로는 착륙점 고정 (너무 늦어서 못 바꿈)
RE_ALIGN_DIST = 0.7        # 새 탐지가 현 착륙점에서 이만큼 떨어지면 재정렬
# 데모용 침입자: 드론이 이 고도 통과할 때 보행자가 착륙 예정지로 걸어 들어옴
INTRUDER_NAME = "person_intruder"
INTRUDER_TRIGGER_ALT = 11.5
INTRUDER_WAIT_XY = (150.0, 9.0)
INTRUDER_WALK_SEC = 5.0
ENABLE_INTRUDER_DEMO = os.environ.get("ENABLE_INTRUDER_DEMO", "1").strip().lower() \
    not in {"0", "false", "no", "off"}

# ── 전방 depth 기반 반응형 회피 (차선/도로 인식 없이 depth만으로 판단) ──
AVOID_RANGE_M = 22.0       # 전방 이 거리 안에 장애물이면 회피 시작
AVOID_SWERVE_M = 9.0       # 회피 시 한 번에 이만큼 측면 오프셋
AVOID_RECOVER_GAIN = 0.05  # 장애물 없을 때 센터라인 복귀 비율 / tick

# ── 교수 피드백 반영: depth 비용지도 + receding-horizon 지역 경로 생성 ──
# legacy: 기존 좌/중/우 오프셋, multistep: 1~3스텝 후보 경로 생성.
LOCAL_PLANNER_MODE = os.environ.get("LOCAL_PLANNER_MODE", "multistep").strip().lower()
LOCAL_PLANNER_HORIZON = max(1, min(3, int(os.environ.get("LOCAL_PLANNER_HORIZON", "2"))))
LOCAL_PLANNER_REPLAN_SEC = float(os.environ.get("LOCAL_PLANNER_REPLAN_SEC", "0.25"))
LOCAL_PLANNER_SENSOR_TIMEOUT_SEC = float(os.environ.get("LOCAL_PLANNER_SENSOR_TIMEOUT_SEC", "0.8"))
LOCAL_PLANNER_RECOVERY_YAW_DEG = float(os.environ.get("LOCAL_PLANNER_RECOVERY_YAW_DEG", "20"))
LOCAL_PLANNER_RECOVERY_MAX = int(os.environ.get("LOCAL_PLANNER_RECOVERY_MAX", "6"))
LOCAL_PLANNER_LOG = os.environ.get("LOCAL_PLANNER_LOG", "")
WAIT_FOR_DESTINATION = os.environ.get("WAIT_FOR_DESTINATION", "0").strip().lower() \
    in {"1", "true", "yes", "on"}

# ── 도시 확장 2단계 (2026-09-03): generate_city.py가 만든 격자 경로(웨이포인트)를 따라
# 실제로 코너를 돌면서 난다. WAYPOINTS_FILE이 없으면 예전과 완전히 동일하게
# [START_XY, TARGET_XY] 직행 2점 경로로 폴백 — 기존 단일 회랑 월드는 아무 변경 없이 그대로 작동.
WAYPOINTS_FILE = os.environ.get("WAYPOINTS_FILE", "")
WAYPOINT_ARRIVE_DIST = 8.0   # 이 거리 안에 들어오면 다음 웨이포인트로 전환


def load_waypoints():
    if WAYPOINTS_FILE and os.path.exists(WAYPOINTS_FILE):
        try:
            with open(WAYPOINTS_FILE) as f:
                data = json.load(f)
            wps = [tuple(p) for p in data["waypoints_xy"]]
            if len(wps) >= 2:
                return wps
        except Exception as e:
            print(f"[mission_controller] WAYPOINTS_FILE 로드 실패({e}) — 직행 경로로 폴백")
    return [START_XY, TARGET_XY]


def dist2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class MissionController(Node):
    def __init__(self):
        super().__init__("mission_controller")
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.pub_status = self.create_publisher(String, "/mission/status", 10)
        self.pub_sp = self.create_publisher(PoseStamped, "/mission/setpoint", 10)
        self.pub_plan_status = self.create_publisher(String, "/planning/status", 10)
        self.pub_local_target = self.create_publisher(PoseStamped, "/planning/local_target", 10)
        self.pub_costmap = self.create_publisher(OccupancyGrid, "/planning/local_costmap", 2)
        self.pub_plan_candidates = self.create_publisher(
            MarkerArray, "/planning/candidates", 2)
        self.pub_selected_path = self.create_publisher(Path, "/planning/selected_path", 2)
        self.sub_target = self.create_subscription(
            PointStamped, "/landing/target", self.on_target, 10)
        self.sub_front_depth = self.create_subscription(
            RosImage, "/drone/front_depth/depth/image_raw", self.on_front_depth, 5)
        # 남은작업 #2 (A안) — RViz "Publish Point" 로 목적지 클릭 지정 (demo.rviz에 툴 추가됨)
        self.sub_click = self.create_subscription(
            PointStamped, "/clicked_point", self.on_click_point, 10)
        self.tf_br = TransformBroadcaster(self)

        self.front_depth = None       # 최근 전방 depth (h, w) float32
        self.front_depth_time = None  # ROS clock seconds — 센서 stale 시 안전 정지
        self.avoid_swerve_y = 0.0     # 현재 회피 측면 오프셋 (목표 y에 더해짐)
        self.last_avoid_log = 0.0     # 로그 throttle

        self.planner_cfg = PlannerConfig(horizon=LOCAL_PLANNER_HORIZON)
        self.last_plan_time = -math.inf
        self.local_plan_target = None       # 선택 경로 첫 점(world xy)
        self.local_plan_yaw = None
        self.latest_plan = None
        self.last_avoid_direction = 1.0
        self.avoidance_active = False
        self.recovery_attempts = 0
        self.last_recovery_time = -math.inf
        self.plan_log_file = None
        if LOCAL_PLANNER_LOG:
            os.makedirs(os.path.dirname(os.path.abspath(LOCAL_PLANNER_LOG)), exist_ok=True)
            self.plan_log_file = open(LOCAL_PLANNER_LOG, "a", encoding="utf-8", buffering=1)

        self.waypoints = load_waypoints()   # [(x,y), ...] — generate_city.py 산출물 또는 직행 2점
        self.default_waypoints = list(self.waypoints)
        self.wp_idx = 1                     # 0번(출발지)은 이미 거기 있으니 1번부터 목표
        self.heading = 0.0                  # 현재 진행 방향(rad) — 회피 오프셋 회전에 씀
        self.target_xy = self.waypoints[-1]  # 최종 목적지 — 클릭으로 재지정 가능 (on_click_point)
        self.pos = [self.waypoints[0][0], self.waypoints[0][1], 7.0]   # 현재 드론 위치 (world)
        self.state = "IDLE"
        self.scan_t0 = None
        self.scan_targets = []              # SCAN 동안 수신한 탐지 좌표들 (중앙값으로 안정화)
        self.landing_target = None          # (x, y, z) — 현재 착륙 목표 (DESCEND 중 갱신될 수 있음)
        self.scan_done = False              # SCAN 끝나 착륙점 1차 확정됨
        self.descend_committed = False      # COMMIT_ALT 아래 — 더 이상 재정렬 안 함
        # 침입자 데모
        self.intruder_active = False
        self.intruder_t0 = None
        self.intruder_from = None
        self.intruder_to = None

        self.get_logger().info("set_entity_state 서비스 대기 중...")
        self.cli.wait_for_service()
        self.get_logger().info("연결됨. 미션 시작.")
        self.get_logger().info(
            f"지역 경로 계획: mode={LOCAL_PLANNER_MODE}, horizon={LOCAL_PLANNER_HORIZON}, "
            f"replan={LOCAL_PLANNER_REPLAN_SEC:.2f}s")
        self.state = "IDLE" if WAIT_FOR_DESTINATION else "TAKEOFF"
        if WAIT_FOR_DESTINATION:
            self.get_logger().info("대시보드 목적지 선택 대기 중 (/clicked_point)")
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

    # ---- 전방 depth 수신 + 반응형 회피 결정 ----
    def on_front_depth(self, msg):
        try:
            self.front_depth = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.width)
            self.front_depth_time = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            self.front_depth = None
            self.front_depth_time = None

    def _publish_costmap(self, costmap, stamp):
        msg = OccupancyGrid()
        msg.header.frame_id = "base_link"
        msg.header.stamp = stamp
        msg.info.resolution = float(costmap.resolution_m)
        msg.info.width = costmap.width
        msg.info.height = costmap.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = float(costmap.y_min_m)
        msg.info.origin.orientation.w = 1.0
        msg.data = costmap.grid.astype(np.int8).ravel().tolist()
        self.pub_costmap.publish(msg)

    @staticmethod
    def _line_marker(marker_id, namespace, rgba, width, stamp):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    def _publish_plan_visuals(self, result, world_paths, selected_world, stamp):
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers = MarkerArray(markers=[delete])
        valid = self._line_marker(1, "valid_candidates", (0.55, 0.65, 0.75, 0.22), 0.045, stamp)
        blocked = self._line_marker(2, "blocked_candidates", (0.95, 0.18, 0.12, 0.20), 0.035, stamp)
        for candidate, world in zip(result.candidates, world_paths):
            dst = blocked if candidate.collision else valid
            for a, b in zip(world[:-1], world[1:]):
                for p in (a, b):
                    point = PointStamped().point
                    point.x, point.y, point.z = float(p[0]), float(p[1]), float(CRUISE_ALT)
                    dst.points.append(point)
        markers.markers.extend([valid, blocked])

        if selected_world is not None:
            chosen = self._line_marker(3, "selected_path", (0.12, 0.95, 0.30, 1.0), 0.20, stamp)
            for a, b in zip(selected_world[:-1], selected_world[1:]):
                for p in (a, b):
                    point = PointStamped().point
                    point.x, point.y, point.z = float(p[0]), float(p[1]), float(CRUISE_ALT + 0.15)
                    chosen.points.append(point)
            markers.markers.append(chosen)

            target = Marker()
            target.header.frame_id = "world"
            target.header.stamp = stamp
            target.ns = "local_target"
            target.id = 4
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position.x = float(selected_world[1, 0])
            target.pose.position.y = float(selected_world[1, 1])
            target.pose.position.z = float(CRUISE_ALT)
            target.pose.orientation.w = 1.0
            target.scale.x = target.scale.y = target.scale.z = 0.75
            target.color.r, target.color.g, target.color.b, target.color.a = 1.0, 0.8, 0.05, 1.0
            target.lifetime.sec = 1
            markers.markers.append(target)

            path = Path()
            path.header.frame_id = "world"
            path.header.stamp = stamp
            for p in selected_world:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(p[0])
                pose.pose.position.y = float(p[1])
                pose.pose.position.z = float(CRUISE_ALT)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.pub_selected_path.publish(path)
        self.pub_plan_candidates.publish(markers)

    def _log_plan(self, record):
        if self.plan_log_file is not None:
            self.plan_log_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def compute_local_plan(self, goal_world_xy):
        """현재 RGB-D 관측 중 depth 기하로 지역 후보 경로를 만들고 첫 점을 저장한다."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_plan_time < LOCAL_PLANNER_REPLAN_SEC:
            return self.latest_plan
        self.last_plan_time = now
        stamp = self.get_clock().now().to_msg()
        if (self.front_depth is None or self.front_depth_time is None or
                now - self.front_depth_time > LOCAL_PLANNER_SENSOR_TIMEOUT_SEC):
            self.local_plan_target = None
            self.local_plan_yaw = None
            self.latest_plan = None
            status = {"status": "NO_SENSOR", "age_s": None if self.front_depth_time is None
                      else round(now - self.front_depth_time, 3)}
            self.pub_plan_status.publish(String(data=json.dumps(status, ensure_ascii=False)))
            self._log_plan({"t": now, "position": self.pos, **status})
            return None

        goal_local = world_to_local(goal_world_xy, self.pos[:2], self.heading)
        t0 = time.perf_counter()
        result = plan_local_path(self.front_depth, goal_local, self.planner_cfg)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        self.latest_plan = result
        selected_world = None
        world_paths = [local_to_world(c.points_xy, self.pos[:2], self.heading)
                       for c in result.candidates]
        if result.selected is not None:
            selected_world = local_to_world(result.selected.points_xy, self.pos[:2], self.heading)
            self.local_plan_target = (float(selected_world[1, 0]), float(selected_world[1, 1]))
            self.local_plan_yaw = math.atan2(self.local_plan_target[1] - self.pos[1],
                                             self.local_plan_target[0] - self.pos[0])
            first_turn = float(result.selected.steering_deg[0])
            obstacle_near = result.selected.min_clearance_m < 8.0
            # 처음 우회를 시작한 쪽을 장애물을 완전히 벗어날 때까지 보존한다.
            # 타워 측면에서 목적지 쪽으로 자세를 되돌리는 조향을 새 우회 방향으로
            # 오인하면 NO_PATH 복구가 장애물 안쪽으로 회전하는 문제가 생긴다.
            if obstacle_near and not self.avoidance_active and abs(first_turn) > 1e-6:
                self.last_avoid_direction = math.copysign(1.0, first_turn)
                self.avoidance_active = True
            elif not obstacle_near:
                self.avoidance_active = False
            self.recovery_attempts = 0
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = stamp
            ps.pose.position.x, ps.pose.position.y = self.local_plan_target
            ps.pose.position.z = CRUISE_ALT
            ps.pose.orientation.w = 1.0
            self.pub_local_target.publish(ps)
        else:
            self.local_plan_target = None
            self.local_plan_yaw = None

        if result.costmap is not None:
            self._publish_costmap(result.costmap, stamp)
        self._publish_plan_visuals(result, world_paths, selected_world, stamp)
        selected = result.selected
        status = {
            "status": result.status,
            "horizon": self.planner_cfg.horizon,
            "plan_ms": round(elapsed_ms, 3),
            "candidate_count": len(result.candidates),
            "valid_count": sum(not c.collision for c in result.candidates),
            "position": [round(v, 3) for v in self.pos],
            "yaw_deg": round(math.degrees(self.heading), 3),
            "goal_world": [round(float(v), 3) for v in goal_world_xy],
            "selected_steering_deg": list(selected.steering_deg) if selected else None,
            "total_cost": round(selected.total_cost, 5) if selected else None,
            "goal_cost": round(selected.goal_cost, 5) if selected else None,
            "clearance_cost": round(selected.clearance_cost, 5) if selected else None,
            "turn_cost": round(selected.turn_cost, 5) if selected else None,
            "unknown_cost": round(selected.unknown_cost, 5) if selected else None,
            "min_clearance_m": round(selected.min_clearance_m, 3) if selected else None,
            "valid_depth_fraction": round(result.costmap.valid_depth_fraction, 5)
            if result.costmap else None,
        }
        self.pub_plan_status.publish(String(data=json.dumps(status, ensure_ascii=False)))
        self._log_plan({"t": now, **status})
        return result

    def try_no_path_recovery(self):
        """제자리 yaw 재관측. 이동하지 않고 새 방향의 depth가 들어온 뒤 다시 계획한다."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_recovery_time < 0.5:
            return False
        if self.recovery_attempts >= LOCAL_PLANNER_RECOVERY_MAX:
            return False
        self.last_recovery_time = now
        self.recovery_attempts += 1
        delta = math.radians(LOCAL_PLANNER_RECOVERY_YAW_DEG) * self.last_avoid_direction
        self.heading = math.atan2(math.sin(self.heading + delta), math.cos(self.heading + delta))
        self.set_pose(self.pos[0], self.pos[1], CRUISE_ALT, yaw=self.heading)
        # 회전 전 관측은 새 카메라 방향의 비용 지도로 사용할 수 없다.
        self.front_depth = None
        self.front_depth_time = None
        self.local_plan_target = None
        self.latest_plan = None
        self.last_plan_time = -math.inf
        event = {
            "status": "RECOVERY_YAW",
            "attempt": self.recovery_attempts,
            "direction": int(self.last_avoid_direction),
            "yaw_deg": round(math.degrees(self.heading), 3),
            "position": [round(v, 3) for v in self.pos],
        }
        self.pub_plan_status.publish(String(data=json.dumps(event, ensure_ascii=False)))
        self._log_plan({"t": now, **event})
        self.get_logger().warn(
            f"지역 경로 없음 → 제자리 재관측 {self.recovery_attempts}/{LOCAL_PLANNER_RECOVERY_MAX} "
            f"(yaw {math.degrees(self.heading):.1f}°)")
        return True

    def compute_avoid_offset(self):
        """전방 depth를 좌/중/우 3등분 → 중앙에 장애물(AVOID_RANGE 안)이면 clearance 큰 쪽으로 swerve."""
        if self.front_depth is None or self.front_depth.size == 0:
            # 데이터 없으면 점진적으로 센터라인 복귀
            self.avoid_swerve_y *= (1.0 - AVOID_RECOVER_GAIN)
            return self.avoid_swerve_y, "no_depth"
        d = self.front_depth
        h, w = d.shape
        # 좌/중/우 3등분 (수평) + 상하 중간 60% 만 (하늘·바닥 제외)
        v1, v2 = int(h * 0.2), int(h * 0.8)
        bw = w // 3
        def safe_pct(s):
            v = s[np.isfinite(s)]
            return float(np.percentile(v, 5)) if v.size > 50 else 200.0
        L = safe_pct(d[v1:v2, :bw])
        C = safe_pct(d[v1:v2, bw:2*bw])
        R = safe_pct(d[v1:v2, 2*bw:])
        # 중앙이 가까우면 회피
        if C < AVOID_RANGE_M:
            if L > R:
                target = +AVOID_SWERVE_M   # +y 쪽으로 (좌)
            else:
                target = -AVOID_SWERVE_M   # -y 쪽으로 (우)
            # smooth approach
            self.avoid_swerve_y += 0.20 * (target - self.avoid_swerve_y)
            log = f"AVOID C={C:.1f}m L={L:.1f} R={R:.1f} → swerve_y={self.avoid_swerve_y:+.1f}"
        else:
            # 장애물 없음 → 센터라인(y=0) 복귀
            self.avoid_swerve_y *= (1.0 - AVOID_RECOVER_GAIN)
            log = f"clear C={C:.1f}m"
        # log throttle
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_avoid_log > 1.5:
            self.get_logger().info(log)
            self.last_avoid_log = now
        return self.avoid_swerve_y, log

    # ---- 목적지 클릭 수신 (RViz Publish Point) ----
    def on_click_point(self, msg: PointStamped):
        if self.state not in ("IDLE", "TAKEOFF", "CRUISE"):
            self.get_logger().warn(
                f"목적지 클릭 무시 (state={self.state} — 도착 절차 시작 후엔 재지정 불가)")
            return
        self.target_xy = (msg.point.x, msg.point.y)
        # 출발 대기 중 선택하면 검증된 기본 도심 경로를 유지하고 마지막 목적지만 붙인다.
        # 이미 비행 중인 재지정은 현재 위치에서 직행하며 depth 지역계획기가 안전을 감시한다.
        if self.state == "IDLE":
            self.waypoints = list(self.default_waypoints)
            if dist2(self.waypoints[-1], self.target_xy) > 0.1:
                self.waypoints.append(self.target_xy)
        else:
            self.waypoints = [tuple(self.pos[:2]), self.target_xy]
        self.wp_idx = 1
        self.avoid_swerve_y = 0.0
        # 새 목적지를 바라본 뒤 그 방향에서 들어온 새 depth로 계획한다.
        self.heading = math.atan2(self.target_xy[1] - self.pos[1],
                                  self.target_xy[0] - self.pos[0])
        self.front_depth = None
        self.front_depth_time = None
        self.local_plan_target = None
        self.latest_plan = None
        self.last_plan_time = -math.inf
        self.set_pose(self.pos[0], self.pos[1], self.pos[2], yaw=self.heading)
        route_kind = "기본 도심 경유 경로" if self.state == "IDLE" else "현재 위치 직행 경로"
        self.get_logger().info(f"[클릭] 새 목적지 지정 ({route_kind}): "
                               f"({self.target_xy[0]:.1f}, {self.target_xy[1]:.1f})")
        if self.state == "IDLE":
            self.state = "TAKEOFF"

    # ---- 착륙지 탐지 결과 수신 ----
    def on_target(self, msg: PointStamped):
        new = (msg.point.x, msg.point.y, max(0.3, msg.point.z))
        if self.state == "SCAN":
            self.scan_targets.append(new)
            return
        # 하강 중 — 아직 commit 전이면, 탐지점이 크게 움직였을 때만 재정렬
        if self.state == "DESCEND" and not self.descend_committed and self.landing_target is not None:
            d = math.hypot(new[0] - self.landing_target[0], new[1] - self.landing_target[1])
            if d > RE_ALIGN_DIST:
                self.get_logger().warn(
                    f"하강 중 장애물 진입 감지 → 재평가 → 착륙점 ({self.landing_target[0]:.1f},{self.landing_target[1]:.1f}) "
                    f"→ ({new[0]:.1f},{new[1]:.1f})")
                self.landing_target = new

    # ---- 드론을 world 좌표 (x,y,z)로 이동 (set_entity_state) ----
    def set_pose(self, x, y, z, yaw=0.0):
        req = SetEntityState.Request()
        req.state.name = DRONE_NAME
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = "world"
        self.cli.call_async(req)   # 비동기 — 응답 안 기다림 (30Hz 유지)
        self.pos = [float(x), float(y), float(z)]
        now = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = now
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = self.pos
        self.pub_sp.publish(ps)
        # TF: world → base_link (드론), base_link → 카메라 링크 (RViz 시각화용)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.pos
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_br.sendTransform(t)
        # base_link → 카메라 링크 (정적이지만 매 틱 보내도 무방). RPY (0, +90°, 0) → 아래를 봄
        for child in ("down_cam_link", "down_depth_link"):
            c = TransformStamped()
            c.header.stamp = now
            c.header.frame_id = "base_link"
            c.child_frame_id = child
            c.transform.translation.z = -0.08
            c.transform.rotation.y = 0.70710678
            c.transform.rotation.w = 0.70710678
            self.tf_br.sendTransform(c)

    def step_toward(self, tx, ty, tz, speed, yaw=0.0):
        """현재 위치에서 (tx,ty,tz)로 한 틱(1/RATE)만큼 이동. 도착하면 True."""
        dx, dy, dz = tx - self.pos[0], ty - self.pos[1], tz - self.pos[2]
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        step = speed / RATE_HZ
        if d <= step:
            self.set_pose(tx, ty, tz, yaw=yaw)
            return True
        k = step / d
        self.set_pose(self.pos[0] + dx * k, self.pos[1] + dy * k, self.pos[2] + dz * k, yaw=yaw)
        return False

    def say(self, s):
        self.pub_status.publish(String(data=s))

    def _move_intruder(self):
        """보행자(person_intruder)를 intruder_from → intruder_to 로 INTRUDER_WALK_SEC 동안 이동."""
        t = (self.get_clock().now() - self.intruder_t0).nanoseconds * 1e-9
        a = min(1.0, t / INTRUDER_WALK_SEC)
        fx, fy = self.intruder_from
        gx, gy = self.intruder_to
        x, y = fx + (gx - fx) * a, fy + (gy - fy) * a
        # 진행 방향으로 yaw 회전
        yaw = math.atan2(gy - fy, gx - fx)
        req = SetEntityState.Request()
        req.state.name = INTRUDER_NAME
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, 0.85
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = "world"
        self.cli.call_async(req)

    # ---- 메인 루프 ----
    def tick(self):
        if self.state == "IDLE":
            self.say("WAITING_FOR_DESTINATION")
            self.set_pose(self.pos[0], self.pos[1], self.pos[2], yaw=self.heading)

        elif self.state == "TAKEOFF":
            self.say("TAKEOFF")
            wx0, wy0 = self.waypoints[0]
            if self.step_toward(wx0, wy0, CRUISE_ALT, SPEED_CRUISE):
                self.state = "CRUISE"
                if len(self.waypoints) > 1:
                    wx1, wy1 = self.waypoints[1]
                    self.heading = math.atan2(wy1 - self.pos[1], wx1 - self.pos[0])
                    self.set_pose(self.pos[0], self.pos[1], self.pos[2], yaw=self.heading)
                    # 이륙 중 yaw=0에서 촬영된 frame은 새 진행방향 계획에 쓰지 않는다.
                    self.front_depth = None
                    self.front_depth_time = None
                self.get_logger().info(
                    f"이륙 완료 → 자율 항법 시작 (경유지 {len(self.waypoints)}개, "
                    f"지역 경로 계획 H{LOCAL_PLANNER_HORIZON})")

        elif self.state == "CRUISE":
            # 웨이포인트를 따라 비행 (차선/도로 인식 없이 좌표만 따라감 — generate_city.py가
            # 오프라인으로 미리 계산한 격자 최단경로, 또는 클릭 시 직행 2점). 코너에서도
            # 전방 depth가 계속 진행방향을 보도록 드론 yaw를 매 틱 진행방향으로 맞춘다.
            wx, wy = self.waypoints[self.wp_idx]
            dx, dy = wx - self.pos[0], wy - self.pos[1]
            dist_to_wp = math.hypot(dx, dy)
            if LOCAL_PLANNER_MODE == "legacy":
                if dist_to_wp > 1e-3:
                    self.heading = math.atan2(dy, dx)
                avoid_mag, _ = self.compute_avoid_offset()
                perp_x, perp_y = -math.sin(self.heading), math.cos(self.heading)
                lookahead = (SPEED_CRUISE / RATE_HZ) * 2.0
                step_x = self.pos[0] + math.cos(self.heading) * lookahead + perp_x * avoid_mag
                step_y = self.pos[1] + math.sin(self.heading) * lookahead + perp_y * avoid_mag
                self.say(f"CRUISE legacy wp{self.wp_idx}/{len(self.waypoints)-1} "
                         f"pos=({self.pos[0]:.0f},{self.pos[1]:.0f})")
                self.step_toward(step_x, step_y, CRUISE_ALT, SPEED_CRUISE, yaw=self.heading)
            else:
                result = self.compute_local_plan((wx, wy))
                if result is None or result.status != "OK" or self.local_plan_target is None:
                    # 센서가 없거나 모든 후보가 막히면 제자리에서 정지한다.
                    reason = "NO_SENSOR" if result is None else result.status
                    recovering = reason == "NO_PATH" and self.try_no_path_recovery()
                    if recovering:
                        self.say(f"CRUISE REOBSERVE {self.recovery_attempts}/"
                                 f"{LOCAL_PLANNER_RECOVERY_MAX}")
                    else:
                        self.say(f"CRUISE STOP — {reason}")
                        self.set_pose(self.pos[0], self.pos[1], CRUISE_ALT, yaw=self.heading)
                else:
                    self.heading = self.local_plan_yaw
                    tx, ty = self.local_plan_target
                    self.say(f"CRUISE H{LOCAL_PLANNER_HORIZON} wp{self.wp_idx}/"
                             f"{len(self.waypoints)-1} cost={result.selected.total_cost:.2f}")
                    self.step_toward(tx, ty, CRUISE_ALT, SPEED_CRUISE, yaw=self.heading)

            if dist_to_wp < WAYPOINT_ARRIVE_DIST:
                if self.wp_idx >= len(self.waypoints) - 1:
                    self.state = "ARRIVE"
                    self.get_logger().info("경로 완주 → 목적지 상공 도착")
                else:
                    self.wp_idx += 1
                    self.avoid_swerve_y = 0.0   # 새 구간 진입 — 회피 오프셋 리셋
                    next_x, next_y = self.waypoints[self.wp_idx]
                    self.heading = math.atan2(next_y - self.pos[1], next_x - self.pos[0])
                    # 제자리에서 새 구간을 바라보고, 이전 방향의 depth는 폐기한다.
                    self.set_pose(self.pos[0], self.pos[1], self.pos[2], yaw=self.heading)
                    self.front_depth = None
                    self.front_depth_time = None
                    self.local_plan_target = None
                    self.latest_plan = None
                    self.last_plan_time = -math.inf
                    self.recovery_attempts = 0
                    self.get_logger().info(f"경유지 통과 → 다음 구간 (wp {self.wp_idx})")

        elif self.state == "ARRIVE":
            self.say("ARRIVE")
            if self.step_toward(self.target_xy[0], self.target_xy[1], SCAN_ALT, SPEED_DESCEND,
                                 yaw=self.heading):
                self.state = "SCAN"
                self.scan_t0 = self.get_clock().now()
                self.get_logger().info(f"스캔 시작 — {SCAN_HOLD_SEC:.0f}s 호버하며 착륙지 탐지")

        elif self.state == "SCAN":
            self.say("SCAN — detecting safe landing zone")
            # 살짝 떠 있게 유지
            self.set_pose(self.target_xy[0], self.target_xy[1], SCAN_ALT)
            elapsed = (self.get_clock().now() - self.scan_t0).nanoseconds * 1e-9
            if elapsed >= SCAN_HOLD_SEC:
                self.scan_done = True
                if self.scan_targets:
                    n = len(self.scan_targets)
                    mx = sorted(t[0] for t in self.scan_targets)[n // 2]
                    my = sorted(t[1] for t in self.scan_targets)[n // 2]
                    tgt = (mx, my, 0.30)
                    src = f"DINOv2 탐지 ({n}회 중앙값)"
                else:
                    tgt = (self.target_xy[0], self.target_xy[1], 0.30)
                    src = "폴백(목적지 좌표)"
                self.landing_target = tgt
                self.get_logger().info(
                    f"착륙 지점 확정 [{src}]: ({tgt[0]:.1f}, {tgt[1]:.1f}, {tgt[2]:.2f}) — 하강 시작")
                self.state = "DESCEND"

        elif self.state == "DESCEND":
            tx, ty, tz = self.landing_target
            tag = " [고정]" if self.descend_committed else ""
            self.say(f"DESCEND → ({tx:.1f},{ty:.1f}) alt={self.pos[2]:.1f}{tag}")
            # 데모: 일정 고도 통과 시 보행자가 착륙 예정지로 걸어 들어옴
            if (ENABLE_INTRUDER_DEMO and not self.intruder_active
                    and self.pos[2] <= INTRUDER_TRIGGER_ALT):
                self.intruder_active = True
                self.intruder_t0 = self.get_clock().now()
                self.intruder_from = INTRUDER_WAIT_XY
                self.intruder_to = (tx, ty)              # 착륙 예정지 정확히 위로 걸어옴
                self.get_logger().info("[데모] 보행자가 착륙 예정지로 접근 중...")
            if self.intruder_active:
                self._move_intruder()
            # COMMIT_ALT 아래로 내려가면 착륙점 고정
            if not self.descend_committed and self.pos[2] <= COMMIT_ALT:
                self.descend_committed = True
                self.get_logger().info(f"착륙점 고정 (alt {COMMIT_ALT}m 이하): ({tx:.1f},{ty:.1f})")
            # 먼저 수평 정렬(현 고도 유지) 후 수직 하강
            if abs(self.pos[0] - tx) > 0.15 or abs(self.pos[1] - ty) > 0.15:
                self.step_toward(tx, ty, self.pos[2], SPEED_DESCEND)
            else:
                if self.step_toward(tx, ty, tz, SPEED_DESCEND):
                    self.state = "LANDED"
                    self.get_logger().info(f"=== 착륙 완료 @ ({tx:.1f},{ty:.1f}) ===")

        elif self.state == "LANDED":
            self.say("LANDED")
            tx, ty, tz = self.landing_target
            self.set_pose(tx, ty, tz)

    def destroy_node(self):
        if self.plan_log_file is not None:
            self.plan_log_file.close()
            self.plan_log_file = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGTERM during an automated recording invalidates the ROS context before
        # the executor returns. Treat only that shutdown race as a clean exit.
        if rclpy.ok():
            raise
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
