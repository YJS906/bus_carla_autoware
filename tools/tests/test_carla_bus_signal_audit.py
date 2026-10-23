import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_bus_signal_audit import crosses_segment


class StopLineCrossingTest(unittest.TestCase):
    def test_crossing_between_samples(self):
        self.assertTrue(crosses_segment((-2, 0), (2, 0), (0, -1), (0, 1)))

    def test_outside_line_is_not_crossing(self):
        self.assertFalse(crosses_segment((-2, 2), (2, 2), (0, -1), (0, 1)))

    def test_parallel_and_stationary(self):
        self.assertFalse(crosses_segment((-1, -2), (-1, 2), (0, -1), (0, 1)))
        self.assertFalse(crosses_segment((0, 0), (0, 0), (0, -1), (0, 1)))

    def test_line_touch_counted_once(self):
        self.assertTrue(crosses_segment((-1, 0), (0, 0), (0, -1), (0, 1)))
        self.assertFalse(crosses_segment((0, 0), (1, 0), (0, -1), (0, 1)))


if __name__ == '__main__':
    unittest.main()
