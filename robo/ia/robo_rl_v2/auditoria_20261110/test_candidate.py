import sys,json
from pathlib import Path
from collections import deque
import numpy as np
p=Path(__file__).parent
sys.path.insert(0,str(p/'candidata'))
from robo_rl_v2.env import RoboEnv
from robo_rl_v2.maps import Grid
checks={}
g=Grid(np.zeros((40,40),np.int8),.05);g.cells[0,:]=1;g.refresh();before=g.cells.copy()
checks['reject_outside']=g.add_box([.01,1],[.5,.5]) is False and np.array_equal(g.cells,before)
checks['reject_wall']=g.add_box([1,.1],[.5,.5]) is False and np.array_equal(g.cells,before)
checks['valid_box']=g.add_box([1,1],[.3,.3]) is True
before=g.cells.copy();checks['reject_overlapping_box']=g.add_box([1,1],[.3,.3]) is False and np.array_equal(g.cells,before)
e=RoboEnv();e.reset(seed=3);e.velocity=np.array([-.01,0.,0.]);e.queue=deque([np.array([.2,0.,0.])]);e.step([1,0,0]);checks['reversal_expected_velocity']=bool(np.isclose(e.velocity[0],.028));checks['reversal_velocity_value']=float(e.velocity[0])
for level in range(1,6):
 e=RoboEnv(level=level,split='eval')
 for i in range(10):e.reset(seed=2500000000+level*100+i);assert not e.collision()
checks['50_valid_resets']=True
assert all(v for k,v in checks.items() if k!='reversal_velocity_value'),checks
(p/'candidate_tests.json').write_text(json.dumps(checks,indent=2));print(checks)
