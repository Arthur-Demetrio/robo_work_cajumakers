import unittest
import numpy as np
from stable_baselines3.common.env_checker import check_env
from robo_rl_v2.arena2016v2 import Arena2016V2Env
from robo_rl_v2.viewer2016v2 import V2Session
class RuntimeTests(unittest.TestCase):
 def test_api_and_reproducibility(self):
  e=Arena2016V2Env(max_steps=4);check_env(e)
  for seed in range(30):
   ob,_=e.reset(seed=seed);pose=e.pose.copy();self.assertFalse(e.collision());self.assertFalse(e.collision(np.r_[e.goal,e.pose[2]]))
   out=e.step([.3,.2,0]);again,_=e.reset(seed=seed);np.testing.assert_array_equal(ob,again);np.testing.assert_array_equal(e.pose,pose);np.testing.assert_array_equal(out[0],e.step([.3,.2,0])[0])
 def test_physical_platform_is_not_lidar_wall(self):
  e=Arena2016V2Env();e.reset(seed=42)
  platform=next(o for o in e.config['objects'] if o['id']=='P06_fundo_direito_1');p=np.array(platform['geometry']['polygon']).mean(0)
  self.assertTrue(e.grid.hits(p));self.assertFalse(e.laser_hits(p));self.assertFalse(e.laser_hits(np.array([[-1,2],[100,100]])).any())
 def test_separate_scene_does_not_add_random_boxes(self):
  s=V2Session(max_steps=2);s.env.reset(seed=42);self.assertEqual(s.env.variant,'arena_2016_v2');self.assertEqual(s.env.config['random_obstacles']['enabled'],False)
if __name__=='__main__':unittest.main()
