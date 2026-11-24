#!/usr/bin/env python3
"""Testes unitarios SEM ROS, mapa real ou motores."""
import math
import unittest
from types import SimpleNamespace
from geometry_recovery_core import (
    Footprint, Grid, Pose, cell_center_base, costmap_sweep, intersects,
    incremental_observability, laser_collision, observability, rotate, sequence,
)


class TestGeometry(unittest.TestCase):
    def setUp(self):
        self.fp = Footprint()
        self.grid = Grid(60, 60, .05, -1.5, -1.5, 0, tuple([0]*3600))

    def test_footprint_is_real_geometry(self):
        self.assertEqual(self.fp.classify(-0.086, 0.307), 'borda_ambigua')
        self.assertEqual(self.fp.classify(0, 0), 'interno')
        self.assertEqual(self.fp.classify(-0.374, 0.319), 'externo')

    def test_sequence(self):
        poses = sequence(-.02, 3)
        self.assertEqual(poses[0], Pose(0, 0, 0))
        self.assertAlmostEqual(poses[-1].x, -.02)
        self.assertAlmostEqual(poses[-1].yaw_deg, 3)
        self.assertTrue(all(p.x <= 0 for p in poses))
        self.assertTrue(all(abs(b.x-a.x) <= .0051 for a,b in zip(poses,poses[1:])))

    def test_sweep_no_obstacle(self):
        result = costmap_sweep(self.grid, self.fp, (0, 0, 0), sequence(-.02, 3))
        self.assertFalse(result['outside'])
        self.assertEqual(result['costs'], {253: 0, 254: 0, 255: 0})

    def test_obstacle_on_rear_right(self):
        # A celula tem x em [-.15,-.10], y em [-.40,-.35].
        data = [0]*3600
        i = 27  # x -0.15 .. -0.10
        j = 22  # y -0.40 .. -0.35
        data[j*60+i] = 254
        grid = Grid(60,60,.05,-1.5,-1.5,0,tuple(data))
        init = costmap_sweep(grid, self.fp, (0,0,0), sequence(0,0))
        positive = costmap_sweep(grid, self.fp, (0,0,0), sequence(0,3))
        negative = costmap_sweep(grid, self.fp, (0,0,0), sequence(0,-3))
        self.assertEqual(init['costs'][254], 0)
        self.assertGreater(positive['costs'][254], 0)
        self.assertEqual(negative['costs'][254], 0)

    def test_laser_obstacle_and_no_obstacle(self):
        point = (-.23, -.33)
        self.assertIsNotNone(laser_collision([point], self.fp, sequence(0,0)))
        self.assertIsNone(laser_collision([(-.38,.32)], self.fp, sequence(-.02,3)))

    def test_grid_origin_rotation(self):
        g = Grid(60,60,.05,2,3,math.pi/2,tuple([0]*3600))
        x,y = g.to_grid_meters(2,4)
        self.assertAlmostEqual(x,1.0)
        self.assertAlmostEqual(y,0.0)

    def test_sat(self):
        a = [(0,0),(1,0),(1,1),(0,1)]
        self.assertTrue(intersects(a, [(1,0),(2,0),(2,1),(1,1)]))
        self.assertFalse(intersects(a, [(1.1,0),(2,0),(2,1),(1.1,1)]))


    def test_baseline_not_counted_as_new_observation(self):
        scan = SimpleNamespace(ranges=[math.inf]*181, angle_increment=math.pi/180,
                               angle_min=-math.pi/2, range_min=.1, range_max=10.)
        result = incremental_observability(scan, (.330, .005, 0), self.fp,
                                            sequence(0, 0))
        self.assertEqual(result['samples'], 0)
        self.assertEqual(result['unknown'], 0)
        moving = incremental_observability(scan, (.330, .005, 0), self.fp,
                                            sequence(-.02, 0))
        self.assertGreater(moving['unknown'], 0)
        self.assertEqual(moving['seen'], 0)

    def test_costmap_cells_and_center(self):
        data = [0]*3600
        data[23*60+25] = 253
        g = Grid(60,60,.05,-1.5,-1.5,0,tuple(data))
        sweep = costmap_sweep(g, self.fp, (0,0,0), sequence(0,0))
        self.assertEqual(sweep['costs'][253], 1)
        self.assertIn((25,23), sweep['cells'][253])
        x,y = cell_center_base(g, (0,0,0), (25,23))
        self.assertAlmostEqual(x,-.225)
        self.assertAlmostEqual(y,-.325)


if __name__ == '__main__':
    unittest.main(verbosity=2)
