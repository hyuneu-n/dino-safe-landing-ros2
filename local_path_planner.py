#!/usr/bin/env python3
"""전방 depth 기반 receding-horizon 지역 경로 계획 코어.

ROS에 의존하지 않는다. Gazebo depth 영상을 드론 로컬 좌표(+x 전방, +y 좌측)의
점유 격자로 바꾸고, 1~3스텝 후보 경로를 생성해 충돌/여유/목적지/회전/미관측
비용으로 평가한다. mission_controller.py는 최적 경로의 첫 점만 실행하고 다음
depth 프레임에서 다시 계획한다.

현재 단계는 교수 피드백 검증을 위한 depth-only 기준 구현이다. DINO 결합 전에도
경로 생성 자체와 horizon 효과를 분리해 측정할 수 있게 모든 비용 성분을 반환한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
import math
from typing import Iterable

import numpy as np


UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)


@dataclass(frozen=True)
class PlannerConfig:
    horizontal_fov_rad: float = 1.5
    min_depth_m: float = 0.3
    max_range_m: float = 24.0
    vertical_min_fraction: float = 0.25
    vertical_max_fraction: float = 0.75
    depth_percentile: float = 5.0
    ray_column_stride: int = 2

    resolution_m: float = 0.5
    lateral_half_width_m: float = 20.0
    safety_radius_m: float = 1.2
    desired_clearance_m: float = 3.0

    horizon: int = 2
    step_length_m: float = 4.0
    steering_deg: tuple[float, ...] = (-40.0, -20.0, 0.0, 20.0, 40.0)
    max_heading_deg: float = 65.0
    path_sample_spacing_m: float = 0.25
    max_candidates: int = 500

    weight_goal: float = 0.45
    weight_clearance: float = 0.30
    weight_turn: float = 0.15
    weight_unknown: float = 0.10

    def __post_init__(self):
        if not 1 <= self.horizon <= 3:
            raise ValueError("horizon은 1~3이어야 합니다")
        if self.resolution_m <= 0 or self.step_length_m <= 0:
            raise ValueError("격자 해상도와 스텝 길이는 양수여야 합니다")
        if not 0 <= self.vertical_min_fraction < self.vertical_max_fraction <= 1:
            raise ValueError("vertical crop 범위가 잘못되었습니다")


@dataclass
class LocalCostmap:
    """grid[y, x]. 원점은 드론이며 x는 전방, y는 좌측이다."""

    grid: np.ndarray
    resolution_m: float
    y_min_m: float
    max_range_m: float
    raw_obstacles_xy: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32))
    valid_depth_fraction: float = 0.0

    @property
    def width(self) -> int:
        return int(self.grid.shape[1])

    @property
    def height(self) -> int:
        return int(self.grid.shape[0])

    def index(self, x_m: np.ndarray, y_m: np.ndarray):
        ix = np.floor(np.asarray(x_m) / self.resolution_m).astype(np.int32)
        iy = np.floor((np.asarray(y_m) - self.y_min_m) / self.resolution_m).astype(np.int32)
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        return ix, iy, inside

    def states_at(self, xy: np.ndarray) -> np.ndarray:
        xy = np.asarray(xy, dtype=np.float32)
        ix, iy, inside = self.index(xy[:, 0], xy[:, 1])
        states = np.full(len(xy), UNKNOWN, dtype=np.int8)
        states[inside] = self.grid[iy[inside], ix[inside]]
        return states


@dataclass
class CandidatePath:
    points_xy: np.ndarray
    steering_deg: tuple[float, ...]
    total_cost: float
    goal_cost: float
    clearance_cost: float
    turn_cost: float
    unknown_cost: float
    min_clearance_m: float
    collision: bool


@dataclass
class PlanResult:
    status: str
    selected: CandidatePath | None
    candidates: list[CandidatePath]
    costmap: LocalCostmap | None
    reason: str = ""


def _finite_depth_values(depth: np.ndarray, cfg: PlannerConfig) -> np.ndarray:
    return np.isfinite(depth) & (depth >= cfg.min_depth_m) & (depth <= cfg.max_range_m)


def build_local_costmap(depth: np.ndarray, cfg: PlannerConfig) -> LocalCostmap:
    """Depth 한 장을 local occupancy costmap으로 변환한다.

    각 영상 열의 중앙 높이 구간에서 가까운 5 percentile을 대표 거리로 사용한다.
    광선의 관측 구간은 FREE, 유한한 반환점은 OCCUPIED로 기록한 뒤 기체 안전 반경만큼
    팽창한다. 반환값이 없는 열은 far clip까지 장애물이 없는 관측으로 취급한다.
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2 or min(depth.shape) < 8:
        raise ValueError("depth는 충분한 크기의 2차원 배열이어야 합니다")
    h, w = depth.shape
    v1 = max(0, min(h - 1, int(round(h * cfg.vertical_min_fraction))))
    v2 = max(v1 + 1, min(h, int(round(h * cfg.vertical_max_fraction))))
    band = depth[v1:v2]
    valid_all = _finite_depth_values(band, cfg)
    valid_fraction = float(valid_all.mean())

    width = int(math.ceil(cfg.max_range_m / cfg.resolution_m)) + 1
    height = int(math.ceil((2.0 * cfg.lateral_half_width_m) / cfg.resolution_m)) + 1
    grid = np.full((height, width), UNKNOWN, dtype=np.int8)
    fx = w / (2.0 * math.tan(cfg.horizontal_fov_rad / 2.0))
    cx = (w - 1) / 2.0

    obstacle_points: list[tuple[float, float]] = []
    free_cells: set[tuple[int, int]] = set()
    y_min = -cfg.lateral_half_width_m

    for u in range(0, w, max(1, cfg.ray_column_stride)):
        col = band[:, u]
        valid = col[np.isfinite(col) & (col >= cfg.min_depth_m) & (col <= cfg.max_range_m)]
        hit = valid.size > 0
        distance = float(np.percentile(valid, cfg.depth_percentile)) if hit else cfg.max_range_m
        distance = max(cfg.min_depth_m, min(cfg.max_range_m, distance))
        # optical u 오른쪽은 body -y, 따라서 local +y(왼쪽)로 부호를 뒤집는다.
        bearing = -math.atan2((u - cx), fx)
        free_end = max(0.0, distance - (cfg.resolution_m if hit else 0.0))
        for radius in np.arange(0.0, free_end + 1e-6, cfg.resolution_m * 0.5):
            x = radius * math.cos(bearing)
            y = radius * math.sin(bearing)
            ix = int(math.floor(x / cfg.resolution_m))
            iy = int(math.floor((y - y_min) / cfg.resolution_m))
            if 0 <= ix < width and 0 <= iy < height:
                free_cells.add((iy, ix))
        if hit and distance < cfg.max_range_m * 0.995:
            obstacle_points.append((distance * math.cos(bearing), distance * math.sin(bearing)))

    for iy, ix in free_cells:
        grid[iy, ix] = FREE

    raw = np.asarray(obstacle_points, dtype=np.float32).reshape(-1, 2)
    if len(raw):
        radius_cells = int(math.ceil(cfg.safety_radius_m / cfg.resolution_m))
        for x, y in raw:
            cx_i = int(math.floor(float(x) / cfg.resolution_m))
            cy_i = int(math.floor((float(y) - y_min) / cfg.resolution_m))
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    ix, iy = cx_i + dx, cy_i + dy
                    if 0 <= ix < width and 0 <= iy < height:
                        grid[iy, ix] = OCCUPIED

    return LocalCostmap(grid, cfg.resolution_m, y_min, cfg.max_range_m, raw, valid_fraction)


def _path_points_from_steering(sequence: Iterable[float], step_m: float,
                               max_heading_deg: float) -> np.ndarray | None:
    x = y = heading = 0.0
    points = [(x, y)]
    for delta_deg in sequence:
        heading += math.radians(float(delta_deg))
        if abs(math.degrees(heading)) > max_heading_deg + 1e-6:
            return None
        x += step_m * math.cos(heading)
        y += step_m * math.sin(heading)
        points.append((x, y))
    return np.asarray(points, dtype=np.float32)


def _sample_polyline(points: np.ndarray, spacing_m: float) -> np.ndarray:
    samples = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        length = float(np.linalg.norm(b - a))
        n = max(1, int(math.ceil(length / spacing_m)))
        samples.extend(a + (b - a) * (i / n) for i in range(1, n + 1))
    return np.asarray(samples, dtype=np.float32)


def _candidate_cost(points: np.ndarray, sequence: tuple[float, ...], goal_xy: np.ndarray,
                    costmap: LocalCostmap, cfg: PlannerConfig) -> CandidatePath:
    samples = _sample_polyline(points, cfg.path_sample_spacing_m)
    # 기체 중심점은 모든 광선의 원점이라 점유 팽창이 겹칠 수 있으므로 첫 0.3m는 제외한다.
    samples_eval = samples[np.linalg.norm(samples, axis=1) >= 0.3]
    states = costmap.states_at(samples_eval)
    collision = bool(np.any(states == OCCUPIED))
    unknown_cost = float(np.mean(states == UNKNOWN)) if len(states) else 1.0

    if len(costmap.raw_obstacles_xy):
        diff = samples_eval[:, None, :] - costmap.raw_obstacles_xy[None, :, :]
        min_clearance = float(np.sqrt(np.sum(diff * diff, axis=2)).min())
    else:
        min_clearance = cfg.max_range_m
    clearance_cost = max(0.0, (cfg.desired_clearance_m - min_clearance) /
                         max(cfg.desired_clearance_m, 1e-6))

    start_goal_dist = max(float(np.linalg.norm(goal_xy)), cfg.step_length_m)
    goal_cost = float(np.linalg.norm(points[-1] - goal_xy)) / start_goal_dist
    max_turn = max(abs(v) for v in cfg.steering_deg) * max(1, len(sequence))
    turn_cost = sum(abs(v) for v in sequence) / max(max_turn, 1e-6)
    total = (cfg.weight_goal * goal_cost +
             cfg.weight_clearance * clearance_cost +
             cfg.weight_turn * turn_cost +
             cfg.weight_unknown * unknown_cost)
    if collision:
        total = math.inf
    return CandidatePath(points, sequence, total, goal_cost, clearance_cost,
                         turn_cost, unknown_cost, min_clearance, collision)


def plan_local_path(depth: np.ndarray, goal_xy_local: tuple[float, float] | np.ndarray,
                    cfg: PlannerConfig | None = None) -> PlanResult:
    """현재 depth와 local-frame 최종/경유 목표로 짧은 후보 경로를 선택한다."""
    cfg = cfg or PlannerConfig()
    try:
        costmap = build_local_costmap(depth, cfg)
    except (TypeError, ValueError) as exc:
        return PlanResult("NO_SENSOR", None, [], None, str(exc))

    goal = np.asarray(goal_xy_local, dtype=np.float32).reshape(2)
    if not np.all(np.isfinite(goal)):
        return PlanResult("NO_GOAL", None, [], costmap, "목표 좌표가 유효하지 않음")

    candidates: list[CandidatePath] = []
    for sequence in product(cfg.steering_deg, repeat=cfg.horizon):
        if len(candidates) >= cfg.max_candidates:
            break
        # 연속해서 좌우 부호가 바뀌는 지그재그는 후보에서 제거한다.
        if any(a * b < 0 for a, b in zip(sequence[:-1], sequence[1:])):
            continue
        points = _path_points_from_steering(sequence, cfg.step_length_m, cfg.max_heading_deg)
        if points is None:
            continue
        candidates.append(_candidate_cost(points, tuple(sequence), goal, costmap, cfg))

    valid = [c for c in candidates if math.isfinite(c.total_cost)]
    if not valid:
        return PlanResult("NO_PATH", None, candidates, costmap,
                          "모든 후보가 장애물 안전 반경과 충돌")
    selected = min(valid, key=lambda c: (c.total_cost, c.turn_cost, c.goal_cost))
    return PlanResult("OK", selected, candidates, costmap)


def local_to_world(points_xy: np.ndarray, origin_xy: tuple[float, float], yaw_rad: float) -> np.ndarray:
    """드론 local(+x 전방,+y 좌측) 경로를 world xy로 변환한다."""
    p = np.asarray(points_xy, dtype=np.float32)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    rot = np.array([[c, -s], [s, c]], dtype=np.float32)
    return p @ rot.T + np.asarray(origin_xy, dtype=np.float32)


def world_to_local(point_xy: tuple[float, float], origin_xy: tuple[float, float],
                   yaw_rad: float) -> np.ndarray:
    """world xy 한 점을 드론 local(+x 전방,+y 좌측) 좌표로 변환한다."""
    delta = np.asarray(point_xy, dtype=np.float32) - np.asarray(origin_xy, dtype=np.float32)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]],
                    dtype=np.float32)
