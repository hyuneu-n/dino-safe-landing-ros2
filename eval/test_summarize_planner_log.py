import unittest

from eval.summarize_planner_log import summarize


class SummarizePlannerLogTest(unittest.TestCase):
    def test_summary(self):
        rows = [
            {"status": "NO_SENSOR", "position": [0, 0, 22]},
            {"status": "RECOVERY_YAW", "position": [0, 0, 22]},
            {"status": "OK", "horizon": 2, "position": [0, 0, 22], "plan_ms": 10,
             "min_clearance_m": 4, "selected_steering_deg": [0, 0]},
            {"status": "OK", "horizon": 2, "position": [3, 4, 22], "plan_ms": 20,
             "min_clearance_m": 2, "selected_steering_deg": [20, 0]},
        ]
        result = summarize(rows)
        self.assertEqual(result["status_counts"], {"NO_SENSOR": 1, "RECOVERY_YAW": 1, "OK": 2})
        self.assertEqual(result["recovery_yaw_count"], 1)
        self.assertEqual(result["turning_plan_count"], 1)
        self.assertEqual(result["logged_route_length_m"], 5)
        self.assertEqual(result["minimum_observed_clearance_m"], 2)


if __name__ == "__main__":
    unittest.main()
