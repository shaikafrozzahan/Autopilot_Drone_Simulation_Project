import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from autopilot_sim import PID, clamp, distance, avoid_obstacles


class TestAutopilotHelpers(unittest.TestCase):
    def test_clamp(self):
        self.assertEqual(clamp(4, -1, 1), 1)
        self.assertEqual(clamp(-4, -1, 1), -1)
        self.assertEqual(clamp(0.5, -1, 1), 0.5)

    def test_distance(self):
        self.assertAlmostEqual(distance((0, 0), (3, 4)), 5.0)

    def test_pid_moves_in_error_direction(self):
        pid = PID(1.0, 0.0, 0.0)
        self.assertGreater(pid.update(2.0, 0.1), 0)

    def test_obstacle_output_is_speed_limited(self):
        vx, vy = avoid_obstacles((5.2, 4.2), (1.5, 0.0))
        self.assertLessEqual((vx * vx + vy * vy) ** 0.5, 1.6 + 1e-9)


if __name__ == "__main__":
    unittest.main()
