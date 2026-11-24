import sys, json, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experimentos_v2_20261110'))
from train_gated import Supervision

class GatedResumeTests(unittest.TestCase):
 def test_plateau_is_optional_and_counter_does_not_stop_at_125000(self):
  for early,expected in [(False,True),(True,False)]:
   with tempfile.TemporaryDirectory() as folder:
    out=Path(folder);(out/'checkpoints').mkdir()
    args=SimpleNamespace(checkpoint_every=25000,eval_every=25000,eval_episodes=1,patience=4,early_stop=early)
    cb=Supervision(out,args)
    model=SimpleNamespace(num_timesteps=125000,save=lambda p:Path(p).write_bytes(b'test'),get_env=lambda:None)
    cb.model=model;cb.num_timesteps=125000;cb.stale=3;cb.best=1.;cb.last_eval=100000
    cb.locals={'dones':[], 'infos':[]}
    row=dict(seed=1,success=False,collision=True,elapsed=1,distance=1,stage=0)
    with patch('train_gated.evaluate',return_value=[row]):self.assertEqual(cb._on_step(),expected)
    self.assertEqual(json.loads((out/'latest.json').read_text())['steps'],125000)
    if not early:
     cb.num_timesteps=model.num_timesteps=150000
     with patch('train_gated.evaluate',return_value=[row]):self.assertTrue(cb._on_step())
     self.assertTrue((out/'checkpoints/ppo_000000150000.zip').exists())
     (out/'stop.request').touch();self.assertFalse(cb._on_step())
    cb.fp.close()
if __name__=='__main__':unittest.main()
