"""Geometry contracts for the unapproved visual candidate, not a fidelity certificate."""
import unittest,importlib.util,json
from pathlib import Path
import numpy as np,cv2
from robo_rl_v2.maps import ROOT,Grid
from robo_rl_v2.env import RoboEnv,FOOTPRINT,rotation
folder=ROOT/'assets/arenas/arena_2016_v2'
spec=importlib.util.spec_from_file_location('arena2016_v2_review',folder/'build_review.py');review=importlib.util.module_from_spec(spec);spec.loader.exec_module(review)
class ReviewTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.cfg=review.load();cls.physical,cls.laser,cls.visual,cls.masks=review.rasterize(cls.cfg)
 def test_floor_and_provenance(self):
  cfg=self.cfg;self.assertEqual(cfg['source']['source_type'],'user_provided_screenshot');self.assertNotEqual(cfg['source']['pdf_original_extraction'],'COMPLETE')
  H=np.array(cfg['floor']['homography_image_to_unit']);corners=np.float32([cfg['floor']['corners_px']]);mapped=cv2.perspectiveTransform(corners,H)[0]
  np.testing.assert_allclose(mapped,[[0,0],[1,0],[1,1],[0,1]],atol=1e-5)
  w,h=cfg['floor']['size_m'];self.assertAlmostEqual(w*h,cfg['floor']['area_m2']);self.assertFalse(cfg['floor']['area_is_official'])
 def test_real_heights_change_lidar_layer(self):
  cfg=dict(self.cfg);base=next(o for o in cfg['objects'] if o['id']=='P06_fundo_direito_1');top=next(o for o in cfg['objects'] if o['kind']=='table_top')
  cfg['objects']=[base];physical,laser,_,_=review.rasterize(cfg);self.assertTrue(physical.any());self.assertFalse(laser.any())
  _,low,_,_=review.rasterize(cfg,height=.08);self.assertTrue(low.any())
  cfg['objects']=[top];physical,laser,visual,_=review.rasterize(cfg);self.assertTrue(visual.any());self.assertFalse(physical.any());self.assertFalse(laser.any())
 def test_complete_footprint_free_states_are_physically_clear(self):
  rng=np.random.default_rng(827);cfg=self.cfg;res=cfg['sensor']['resolution_m'];env=RoboEnv();env.grid=Grid(self.physical.astype(np.int8),res);w,h=cfg['floor']['size_m']
  tested=0
  for yaw in np.linspace(0,2*np.pi,12,endpoint=False):
   free=review.cspace(self.physical,yaw,res);ys,xs=np.nonzero(free)
   for k in rng.choice(len(xs),size=50,replace=False):
    pose=np.array([(xs[k]+.5)*res,(ys[k]+.5)*res,yaw]);self.assertFalse(env.collision(pose,margin=.01))
    corners=FOOTPRINT@rotation(yaw).T+pose[:2];self.assertTrue((corners>=0).all());self.assertTrue((corners<=np.array([w,h])).all());tested+=1
  self.assertEqual(tested,600)
 def test_service_poses_and_unresolved_cases_are_reported(self):
  services=json.loads((folder/'review/service_regions.json').read_text());report=json.loads((folder/'review/navigation_validation.json').read_text())
  env=RoboEnv();env.grid=Grid(self.physical.astype(np.int8),self.cfg['sensor']['resolution_m'])
  for s in services:
   if s['accessible']:
    self.assertFalse(env.collision(s['pose'],margin=.01));self.assertEqual(s['approach_width_m'],.8)
   else:self.assertIn(s['object'],report['unresolved_services'])
  self.assertFalse(report['all_navigation_requirements_satisfied']);self.assertEqual(report['random_obstacles_generated'],0)
if __name__=='__main__':unittest.main()
