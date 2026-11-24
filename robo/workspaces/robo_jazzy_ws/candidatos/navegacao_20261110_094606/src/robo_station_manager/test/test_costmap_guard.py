import math
import unittest
from robo_station_manager.costmap_guard import footprint_is_clear


class TestFootprintGuard(unittest.TestCase):
    def grid(self, fill=0):
        return [fill] * (100 * 100)

    def check(self, arr, robot_yaw=0., **kwargs):
        return footprint_is_clear(arr, 100, 100, 0.05, 0., 0., 0.,
                                  2.5, 2.5, robot_yaw, **kwargs)

    def test_clear(self):
        self.assertTrue(self.check(self.grid())[0])

    def test_block_at_front(self):
        arr = self.grid()
        arr[50 * 100 + 56] = 100
        ok, _ = self.check(arr)
        self.assertFalse(ok)

    def test_unknown_inside(self):
        arr = self.grid()
        arr[50 * 100 + 50] = -1
        self.assertFalse(self.check(arr)[0])

    def test_rotated_footprint(self):
        arr = self.grid()
        arr[56 * 100 + 50] = 100
        ok, _ = self.check(arr, robot_yaw=math.pi / 2)
        self.assertFalse(ok)

    def test_outside_map_fails(self):
        ok, _ = footprint_is_clear(self.grid(),100,100,0.05,0,0,0,0.1,0.1,0)
        self.assertFalse(ok)

    def test_origin_rotation(self):
        arr = self.grid()
        ok, _ = footprint_is_clear(arr,100,100,0.05,5,5,math.pi / 2,7.5,7.5,0.)
        self.assertFalse(ok)  # fora do grid rotacionado

    def test_corrupt_map(self):
        ok, _ = self.check([0])
        self.assertFalse(ok)
