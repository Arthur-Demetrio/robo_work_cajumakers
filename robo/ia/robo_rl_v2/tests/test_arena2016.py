import unittest
from pathlib import Path
import numpy as np
from robo_rl_v2.arena2016 import Arena2016Env,read_layout,rasterize,validate,LAYOUT
from robo_rl_v2.env import RADIUS
from robo_rl_v2.maps import Grid
class ArenaTests(unittest.TestCase):
 def test_raster_and_gaps(self):
  c=read_layout();g,m=rasterize(c);v=validate(c,g,m)
  self.assertEqual(g.cells.shape,(400,480));self.assertEqual(v['violations'],[]);self.assertGreater(g.n_components,0)
  for variant in c['variants']:np.testing.assert_array_equal(Grid.load(LAYOUT.parent/'generated'/f'{variant}.yaml').cells,g.cells)
 def test_variants_and_reproducibility(self):
  for variant in read_layout()['variants']:
   e=Arena2016Env(variant=variant)
   for i in range(12):
    ob,info=e.reset(seed=3100000000+i);cells=e.grid.cells.copy();pose=e.pose.copy();goal=e.goal.copy()
    self.assertFalse(e.collision());self.assertGreater(e.grid.distance(e.goal),RADIUS);self.assertEqual(info['boxes'],3 if variant.endswith('treino') else 0)
    si,gi=e.grid.indices([e.pose[:2],e.goal]);self.assertEqual(e.grid.components[si[1],si[0]],e.grid.components[gi[1],gi[0]])
    again,_=e.reset(seed=3100000000+i)
    for a,b in [(ob,again),(cells,e.grid.cells),(pose,e.pose),(goal,e.goal)]:np.testing.assert_array_equal(a,b)
   e.close()
 def test_noise_off_and_progression(self):
  e=Arena2016Env(variant='arena_2016_treino',noise=False,progression=True);e.set_curriculum([10,20,30,40])
  for transitions,level,count in [(0,1,0),(10,2,0),(20,3,1),(30,4,2),(40,5,3)]:
   e.transitions=transitions;_,info=e.reset(seed=33);self.assertEqual(e.level,level);self.assertEqual(info['boxes'],count);self.assertFalse(info['noise']);self.assertEqual(e.sensor_noise,0.)
  e=Arena2016Env();e.reset(seed=12);pose=e.pose.copy();e.step([0,0,0]);np.testing.assert_array_equal(e.pose,pose)
 def test_reject_bad_geometry(self):
  c=read_layout();c['fixtures'].append(dict(id='impossible',kind='platform',shapes=[dict(rect=[.2,.2,.2,.2])]))
  g,m=rasterize(c)
  with self.assertRaises(ValueError):validate(c,g,m)
if __name__=='__main__':unittest.main()
