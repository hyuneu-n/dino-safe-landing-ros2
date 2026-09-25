"""Depth-only 지역 경로 계획 코어의 결정적 단위 시험."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_path_planner import (  # noqa: E402
    PlannerConfig, local_to_world, plan_local_path, world_to_local,
)


def scene(default=80.0):
    return np.full((240, 320), default, dtype=np.float32)


class LocalPathPlannerTest(unittest.TestCase):
    def config(self, **kwargs):
        base = dict(horizon=2, max_range_m=24.0, step_length_m=4.0,
                    steering_deg=(-40.0, -20.0, 0.0, 20.0, 40.0))
        base.update(kwargs)
        return PlannerConfig(**base)

    def test_clear_scene_selects_straight_path(self):
        result = plan_local_path(scene(), (30.0, 0.0), self.config())
        self.assertEqual(result.status, "OK")
        self.assertEqual(result.selected.steering_deg, (0.0, 0.0))
        self.assertAlmostEqual(float(result.selected.points_xy[-1, 1]), 0.0, places=5)

    def test_center_obstacle_selects_a_detour(self):
        depth = scene()
        depth[60:180, 130:190] = 6.0
        result = plan_local_path(depth, (30.0, 0.0), self.config())
        self.assertEqual(result.status, "OK")
        self.assertFalse(result.selected.collision)
        self.assertNotEqual(result.selected.steering_deg[0], 0.0)
        self.assertGreater(abs(float(result.selected.points_xy[1, 1])), 0.5)

    def test_close_wall_returns_no_path(self):
        depth = np.full((240, 320), 1.0, dtype=np.float32)
        result = plan_local_path(depth, (30.0, 0.0), self.config())
        self.assertEqual(result.status, "NO_PATH")
        self.assertIsNone(result.selected)

    def test_horizon_candidate_sets_are_bounded(self):
        for horizon in (1, 2, 3):
            with self.subTest(horizon=horizon):
                result = plan_local_path(scene(), (30.0, 0.0), self.config(horizon=horizon))
                self.assertEqual(result.status, "OK")
                self.assertLessEqual(len(result.candidates), 5 ** horizon)
                self.assertGreater(len(result.candidates), 0)

    def test_world_local_coordinate_round_trip(self):
        origin = (10.0, -3.0)
        yaw = math.radians(37.0)
        world = (18.0, 4.0)
        local = world_to_local(world, origin, yaw)
        restored = local_to_world(local[None, :], origin, yaw)[0]
        np.testing.assert_allclose(restored, world, atol=1e-5)

    def test_invalid_depth_stops(self):
        result = plan_local_path(np.zeros((2, 2), np.float32), (10.0, 0.0), self.config())
        self.assertEqual(result.status, "NO_SENSOR")


if __name__ == "__main__":
    unittest.main()
