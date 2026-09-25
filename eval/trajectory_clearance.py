#!/usr/bin/env python3
"""계획 로그 궤적과 SDF의 명시적 box/cylinder 충돌체 사이 최소 여유 계산."""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _pose(text):
    values = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    return (values + [0.0] * 6)[:6]


def load_obstacles(world_path, flight_z=22.0, drone_half_height=0.25):
    root = ET.parse(world_path)
    obstacles = []
    for model in root.findall(".//world/model"):
        if model.get("name") == "delivery_drone":
            continue
        mp = _pose(model.findtext("pose"))
        for collision in model.findall("./link/collision"):
            cp = _pose(collision.findtext("pose"))
            yaw = mp[5] + cp[5]
            x = mp[0] + cp[0] * math.cos(mp[5]) - cp[1] * math.sin(mp[5])
            y = mp[1] + cp[0] * math.sin(mp[5]) + cp[1] * math.cos(mp[5])
            z = mp[2] + cp[2]
            size = collision.findtext("./geometry/box/size")
            radius = collision.findtext("./geometry/cylinder/radius")
            if size:
                sx, sy, sz = map(float, size.split())
                if z + sz / 2 < flight_z - drone_half_height or z - sz / 2 > flight_z + drone_half_height:
                    continue
                obstacles.append({"name": model.get("name"), "kind": "box", "x": x, "y": y,
                                  "yaw": yaw, "sx": sx, "sy": sy})
            elif radius:
                length = float(collision.findtext("./geometry/cylinder/length"))
                if z + length / 2 < flight_z - drone_half_height or z - length / 2 > flight_z + drone_half_height:
                    continue
                obstacles.append({"name": model.get("name"), "kind": "cylinder", "x": x, "y": y,
                                  "radius": float(radius)})
    return obstacles


def _signed_surface_distance(point, obstacle):
    px, py = float(point[0]), float(point[1])
    if obstacle["kind"] == "cylinder":
        return math.hypot(px - obstacle["x"], py - obstacle["y"]) - obstacle["radius"]
    c, s = math.cos(obstacle["yaw"]), math.sin(obstacle["yaw"])
    dx, dy = px - obstacle["x"], py - obstacle["y"]
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    qx, qy = abs(lx) - obstacle["sx"] / 2, abs(ly) - obstacle["sy"] / 2
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    if qx <= 0 and qy <= 0:
        return -min(-qx, -qy)
    return outside


def sample_trajectory(positions, spacing_m=0.1):
    if not positions:
        return np.empty((0, 2), np.float32)
    samples = [np.asarray(positions[0][:2], np.float32)]
    for a, b in zip(positions[:-1], positions[1:]):
        a = np.asarray(a[:2], np.float32)
        b = np.asarray(b[:2], np.float32)
        distance = float(np.linalg.norm(b - a))
        n = max(1, int(math.ceil(distance / spacing_m)))
        samples.extend(a + (b - a) * (i / n) for i in range(1, n + 1))
    return np.asarray(samples, np.float32)


def audit_trajectory(positions, world_path, flight_z=22.0, drone_radius_m=0.36,
                     spacing_m=0.1):
    obstacles = load_obstacles(world_path, flight_z)
    samples = sample_trajectory(positions, spacing_m)
    if not len(samples) or not obstacles:
        return {"explicit_obstacle_count": len(obstacles), "trajectory_sample_count": len(samples),
                "minimum_surface_clearance_m": None, "nearest_obstacle": None,
                "collision_sample_count": 0}
    minimum = math.inf
    nearest = None
    collision_samples = 0
    for point in samples:
        distances = [(_signed_surface_distance(point, obstacle), obstacle["name"])
                     for obstacle in obstacles]
        distance, name = min(distances)
        if distance < minimum:
            minimum, nearest = distance, name
        if distance < drone_radius_m:
            collision_samples += 1
    return {
        "explicit_obstacle_count": len(obstacles),
        "trajectory_sample_count": len(samples),
        "minimum_surface_clearance_m": minimum,
        "nearest_obstacle": nearest,
        "drone_radius_m": drone_radius_m,
        "collision_sample_count": collision_samples,
        "collision_free": collision_samples == 0,
    }

