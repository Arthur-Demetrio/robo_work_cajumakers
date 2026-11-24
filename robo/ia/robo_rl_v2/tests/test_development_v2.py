import sys,unittest,json
from pathlib import Path
import numpy as np,yaml
from robo_rl_v2.maps import ROOT
CODE=ROOT/'experimentos_v2_20261110';sys.path.insert(0,str(CODE))
from envs import FastV2Env
from native import NativeV2Env
class DevelopmentTests(unittest.TestCase):
 def test_native_preserves_dynamics_and_sensors(self):
  a=FastV2Env();b=NativeV2Env()
  for seed in [33,34,35]:
   x,_=a.reset(seed=seed);y,_=b.reset(seed=seed);np.testing.assert_array_equal(x,y)
   for action in np.random.default_rng(seed).uniform(-1,1,(20,3)):
    x=a.step(action);y=b.step(action);np.testing.assert_array_equal(x[0],y[0]);self.assertEqual(x[1:4],y[1:4]);self.assertEqual(x[4],y[4])
    if x[2] or x[3]:break
 def test_waypoints_use_estimate_not_true_pose(self):
  e=FastV2Env(guidance=True);e.reset(seed=55);a=e.observation();e.pose[:2]+=[1.,2.];b=e.observation();np.testing.assert_array_equal(a,b)
  self.assertTrue(all(e.grid.distance(p)>.52 for p in e.route))
 def test_all_service_candidates_keep_objects_and_height(self):
  original=yaml.safe_load((ROOT/'assets/arenas/arena_2016_v2/layout_review.yaml').read_text());candidate=yaml.safe_load((CODE/'layout_service_candidate.yaml').read_text())
  self.assertEqual(original['objects'],candidate['objects']);self.assertEqual(original['sensor'],candidate['sensor']);self.assertEqual(original['robot'],candidate['robot'])
  e=FastV2Env();e.reset(seed=42)
  for s in candidate['service_regions']:
   self.assertTrue(s['accessible']);self.assertGreaterEqual(s['approach_width_m'],.8);self.assertFalse(e.collision(np.array(s['pose']),margin=.01))
  self.assertEqual(len(candidate['service_regions']),14)
if __name__=='__main__':unittest.main()
