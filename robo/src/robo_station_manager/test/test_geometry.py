import math
import unittest

from robo_station_manager.geometry import Station, compute_approach, path_length_xy


class TestGeometry(unittest.TestCase):
    def test_table_in_front_pointing_up(self):
        # Bancada em y=2; lado livre é y<2. Robô deve apontar para +Y.
        a = compute_approach(Station('A', (0.0, 2.0), (0.8, 2.0), (0.4, 0.0)))
        self.assertAlmostEqual(a.base_x, 0.4)
        self.assertAlmostEqual(a.base_y, 2.0-0.565)
        self.assertAlmostEqual(a.yaw, math.pi / 2)
        self.assertAlmostEqual(a.width, 0.8)

    def test_table_on_left_pointing_right(self):
        a = compute_approach(Station('B', (2., 0.), (2., 0.8), (3.,0.4)))
        self.assertAlmostEqual(a.base_x, 2.565)
        self.assertAlmostEqual(a.base_y, 0.4)
        self.assertAlmostEqual(abs(a.yaw), math.pi)

    def test_order_invariant(self):
        p = compute_approach(Station('C', (1.,1.), (2.,1.), (1.5, 0.)))
        q = compute_approach(Station('C', (2.,1.), (1.,1.), (1.5, 0.)))
        self.assertAlmostEqual(p.base_x, q.base_x)
        self.assertAlmostEqual(p.base_y, q.base_y)
        self.assertAlmostEqual(p.yaw, q.yaw)

    def test_degenerate_edge(self):
        with self.assertRaises(ValueError):
            compute_approach(Station('A', (0.,0.), (0.01,0.), (1.,0.)))

    def test_free_point_on_edge(self):
        with self.assertRaises(ValueError):
            compute_approach(Station('A', (0.,0.), (1.,0.), (0.5,0.05)))

    def test_length(self):
        self.assertAlmostEqual(path_length_xy([(0,0), (3,4), (6,8)]), 10.)


if __name__ == '__main__':
    unittest.main()
