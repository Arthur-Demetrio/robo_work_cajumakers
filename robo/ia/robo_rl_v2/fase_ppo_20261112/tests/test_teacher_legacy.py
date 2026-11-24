import unittest,tempfile,json,sys
from pathlib import Path
import numpy as np
from robo_rl_v2.teacher_legacy.adapter import load_core,points_from_scan,command_label,DemonstrationWriter
class LegacyTeacherTests(unittest.TestCase):
 def test_actual_core_loads_without_ros(self):
  before='rclpy' in sys.modules;c=load_core();self.assertTrue(hasattr(c,'Encosto'));self.assertTrue(hasattr(c,'TravessiaLateral'));self.assertEqual('rclpy' in sys.modules,before)
 def test_actual_encosto_stops_when_measurements_are_stale(self):
  c=load_core();e=c.Encosto(log=lambda *_:None);e.t0=0.;e.estado='girar';e.t_estado=0.;e.ini=(0,0,0)
  self.assertEqual(e.passo(2.,(0,0,0)),(0.,0.,0.));self.assertNotEqual(e.estado,'girar')
 def test_actual_corridor_measurement(self):
  c=load_core();pts=[(x,.41) for x in np.linspace(.2,1.4,40)]+[(x,-.41) for x in np.linspace(.2,1.4,40)]
  r=c.medir_corredor(pts);self.assertAlmostEqual(r['largura'],.82,places=5);self.assertAlmostEqual(r['desvio'],0.,places=5)
 def test_unrepresentable_turn_is_not_clipped(self):
  a=command_label((0,0,.30));self.assertFalse(a['representable']);self.assertGreater(a['action'][2],1.);self.assertAlmostEqual(command_label((-.08,0,0))['action'][0],-.5)
 def test_scan_frame_and_dataset(self):
  points=points_from_scan([1,1,0],[0,np.pi/2,0],[True,False,True]);self.assertAlmostEqual(points[0][0],1.33)
  with tempfile.TemporaryDirectory() as tmp:
   path=Path(tmp)/'demos.jsonl'
   with DemonstrationWriter(path) as w:w.append(np.zeros(152),[.02,0,0],'encosto','avancar',1.)
   row=json.loads(path.read_text());self.assertTrue(row['teacher']['representable'])
   with self.assertRaises(FileExistsError):DemonstrationWriter(path)
if __name__=='__main__':unittest.main()
