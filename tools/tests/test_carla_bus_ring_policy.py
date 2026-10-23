import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_bus_ring_policy import RingPolicy


class RingPolicyTest(unittest.TestCase):
    def test_sequential_merges_and_no_curve_requests(self):
        p = RingPolicy()
        self.assertIsNone(p.observe(30, -2, 0, 0, 0))
        self.assertEqual(p.observe(40, -2, 0, 0, 2), -3)
        p.accepted(-3)
        p.observe(75, -3, -.4, 0, 6)
        self.assertIsNone(p.observe(85, -3, -.4, 0, 8))
        self.assertEqual(p.cap_kmh(85), 50)
        for s in (150, 250, 500, 600, 657):
            self.assertIsNone(p.observe(s, -3, -.6, .01, s))
        self.assertEqual(p.cap_kmh(625), 20)
        self.assertEqual(p.observe(670, -3, -.4, 0, 670), -2)
        p.accepted(-2)
        self.assertIsNone(p.observe(695, -3, -.4, 0, 675))
        p.observe(710, -2, 0, 0, 680)
        self.assertEqual(p.observe(720, -2, 0, 0, 682), -1)
        p.accepted(-1)
        p.observe(750, -1, .4, 0, 690)
        p.observe(760, -1, .4, 0, 692)
        self.assertEqual(p.stage, 'exit_ready')

    def test_incomplete_entry_cannot_continue_into_curve(self):
        p = RingPolicy()
        p.accepted(-3)
        with self.assertRaisesRegex(RuntimeError, 'before ring curve'):
            p.observe(130, -2, 0, 0, 40)

    def test_not_stable_during_lateral_motion(self):
        p = RingPolicy()
        for t in range(5):
            self.assertIsNone(p.observe(40+t, -2, 1.1, .1, t))


if __name__ == '__main__':
    unittest.main()
