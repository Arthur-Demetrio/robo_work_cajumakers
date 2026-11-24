import sys
from pathlib import Path
from types import SimpleNamespace as S
import unittest
sys.path.insert(0, str(Path(__file__).parent / 'payload/robo_progressive_spin/robo_progressive_spin'))
from geometry import spin_options, select_spin, arc_clear

FP=[(.395,.345),(.395,-.345),(-.235,-.345),(-.235,.345)]
def grid(points=(), unknown=False):
    n=160; res=.02
    g=S(info=S(width=n,height=n,resolution=res,origin=S(position=S(x=-1.6,y=-1.6),orientation=S(x=0,y=0,z=0,w=1))),data=[0]*(n*n))
    for x,y in points:
        ix=int((x+1.6)/res); iy=int((y+1.6)/res)
        g.data[iy*n+ix]=-1 if unknown else 100
    return g

class Tests(unittest.TestCase):
    def test_corner_left(self):
        g=grid([(.38,.38),(.40,.38),(.42,.38)])
        result=select_spin(spin_options(g,(0,0,0),FP,.12,.36,.20))
        self.assertIsNotNone(result)
        self.assertEqual(result['sign'],-1)
        self.assertTrue(arc_clear(g,(0,0,0),FP,-.12))
    def test_corner_right(self):
        g=grid([(.38,-.38),(.40,-.38),(.42,-.38)])
        result=select_spin(spin_options(g,(0,0,0),FP,.12,.36,.20))
        self.assertEqual(result['sign'],1)
    def test_unknown(self):
        self.assertEqual(spin_options(grid([(0,0)],True),(0,0,0),FP,.12,.36,.20),[])
    def test_both_blocked(self):
        self.assertEqual(spin_options(grid([(.34,.36),(.34,-.36)]),(0,0,0),FP,.12,.36,.20),[])
    def test_no_reversal(self):
        options=spin_options(grid(),(0,0,0),FP,.12,.36,.20)
        self.assertEqual(select_spin(options,-1)['sign'],-1)
        self.assertIsNone(select_spin([o for o in options if o['sign']==1],-1))
    def test_open_tie(self):
        options=spin_options(grid(),(0,0,0),FP,.12,.36,.20)
        self.assertEqual(select_spin(options,preferred=-1)['sign'],-1)
    def test_map_boundary(self):
        self.assertEqual(spin_options(grid(),(1.5,0,0),FP,.12,.36,.20),[])

if __name__=='__main__': unittest.main()
