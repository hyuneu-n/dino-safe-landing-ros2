import tempfile
from pathlib import Path
import unittest

from eval.trajectory_clearance import audit_trajectory


WORLD = """<sdf version='1.6'><world name='w'>
<model name='wall'><static>true</static><pose>5 0 2 0 0 0</pose><link name='l'>
<collision name='c'><geometry><box><size>2 2 6</size></box></geometry></collision>
</link></model></world></sdf>"""


class TrajectoryClearanceTest(unittest.TestCase):
    def test_detects_collision_and_clear_route(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = Path(tmp) / "test.world"
            world.write_text(WORLD)
            hit = audit_trajectory([(0, 0, 2), (10, 0, 2)], world, flight_z=2)
            clear = audit_trajectory([(0, 3, 2), (10, 3, 2)], world, flight_z=2)
            self.assertFalse(hit["collision_free"])
            self.assertTrue(clear["collision_free"])
            self.assertAlmostEqual(clear["minimum_surface_clearance_m"], 2.0, places=5)


if __name__ == "__main__":
    unittest.main()
