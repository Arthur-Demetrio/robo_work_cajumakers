import json,shutil,tempfile,unittest
from pathlib import Path
from robo_rl_v2.viewer2016 import latest_2016_checkpoint,ArenaEvaluationSession
from robo_rl_v2.viewer_session import load_policy
from robo_rl_v2.maps import ROOT
RUN=ROOT/'runs/treino_2016_transferido_20261110_085342'
class Viewer2016Tests(unittest.TestCase):
 def test_partial_zip_falls_back(self):
  complete=latest_2016_checkpoint(RUN)
  with tempfile.TemporaryDirectory() as tmp:
   run=Path(tmp);(run/'checkpoints').mkdir()
   shutil.copy2(complete.path,run/'checkpoints'/complete.path.name)
   (run/'checkpoints/ppo_999999999999.zip').write_bytes(b'partial checkpoint')
   self.assertEqual(latest_2016_checkpoint(run).steps,complete.steps)
 def test_new_arena_and_fixed_stage(self):
  checkpoint=latest_2016_checkpoint(RUN);model=load_policy(checkpoint)
  s=ArenaEvaluationSession(RUN,max_steps=2);s.activate(checkpoint,model)
  self.assertEqual(s.env.grid.cells.shape,(400,480));self.assertEqual(s.env.variant,'arena_2016_treino')
  expected=min(5,1+sum(checkpoint.steps>=int(s.config['steps']*f) for f in [.15,.35,.55,.8]));self.assertEqual(s.env.level,expected)
  pose=s.env.pose.copy();s.step();s.restart()
  import numpy as np
  np.testing.assert_array_equal(pose,s.env.pose);self.assertEqual(s.env.transitions,checkpoint.steps)
  self.assertFalse(model.policy.training);s.env.close()
if __name__=='__main__':unittest.main()
