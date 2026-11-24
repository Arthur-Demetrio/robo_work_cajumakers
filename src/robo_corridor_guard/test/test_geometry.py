import math
import random
import unittest
from robo_corridor_guard.geometry import find_corridor, preview_cmd,Config


def make_walls(width=.80, shift=0.0, slope=0, n=25, noise=0):
    rng=random.Random(4)
    out=[]
    for i in range(n):
        x=-.22 + .90*i/(n-1)
        out.extend([(x,shift+width/2+slope*x+rng.uniform(-noise,noise)),
                    (x,shift-width/2+slope*x+rng.uniform(-noise,noise))])
    return out

class CorridorTests(unittest.TestCase):
    def test_80cm_corridor(self):
        res=find_corridor(make_walls())
        self.assertTrue(res.detected)
        self.assertAlmostEqual(res.width_m,.8,places=3)
        self.assertAlmostEqual(res.physical_clearance_m,.085,places=3)
    def test_80cm_corridor_noise(self):
        res=find_corridor(make_walls(noise=.01))
        self.assertTrue(res.detected)
        self.assertAlmostEqual(res.width_m,.8,delta=.02)
    def test_one_wall_not_corridor(self):
        res=find_corridor([(x,y) for x,y in make_walls() if y>0])
        self.assertFalse(res.detected)
    def test_wall_angle(self):
        res=find_corridor(make_walls(slope=.12))
        self.assertTrue(res.detected)
        self.assertAlmostEqual(res.heading_rad,math.atan(.12),delta=.02)
    def test_narrow_or_wall_offset_stops(self):
        res=find_corridor(make_walls(shift=.06))
        self.assertTrue(res.detected)
        self.assertEqual(res.reason,'FOLGA_INSUFICIENTE')
        _,reason=preview_cmd(.04,0,0,res,True)
        self.assertEqual(reason,'FOLGA_INSUFICIENTE')
    def test_reverse_turn_guard(self):
        c=find_corridor(make_walls())
        vals,reason=preview_cmd(-.05,0,.09,c,True)
        self.assertEqual(vals,(0.,0.,0.))
        self.assertEqual(reason,'RE_COM_ROTACAO')
    def test_lateral_motion_guard(self):
        c=find_corridor(make_walls())
        vals,reason=preview_cmd(0,.03,0,c,True)
        self.assertEqual(vals,(0.,0.,0.))
        self.assertEqual(reason,'DESLOCAMENTO_LATERAL_NO_CORREDOR')
    def test_rotate_guard(self):
        c=find_corridor(make_walls())
        _,reason=preview_cmd(.05,0,.1,c,True)
        self.assertEqual(reason,'GIRO_EXCESSIVO')
    def test_forward_straight_preview(self):
        c=find_corridor(make_walls())
        vals,reason=preview_cmd(.04,0,0,c,True)
        self.assertEqual(vals,(.04,0,0))
        self.assertEqual(reason,'PASSAGEM_CANDIDATA_SEM_BLOQUEIO_DE_COMANDO')
    def test_stale_scan_blocks_preview(self):
        vals,reason=preview_cmd(.05,0,0,find_corridor([]),False)
        self.assertEqual(vals,(0.,0.,0.))
        self.assertEqual(reason,'SCAN_AUSENTE_OU_ANTIGO')
    def test_no_corridor_outside(self):
        vals,reason=preview_cmd(.05,0,.04,find_corridor([]),True)
        self.assertEqual(vals,(.05,0,.04))
        self.assertEqual(reason,'FORA_DO_MODO_CORREDOR')

if __name__=='__main__': unittest.main()
