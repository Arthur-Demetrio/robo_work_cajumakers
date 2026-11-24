from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
import json
import numpy as np
from stable_baselines3.common.env_checker import check_env
from gymnasium.utils.env_checker import check_env as gym_check
from scipy.ndimage import distance_transform_edt
from robo_rl_v2.arena2016 import Arena2016Env,read_layout
from robo_rl_v2.visual import draw,plt
out=Path(__file__).parent
rows=[]
for variant in read_layout()['variants']:
 e=Arena2016Env(variant=variant,max_steps=5);check_env(e);gym_check(e,skip_render_check=True)
 for i in range(100):
  _,info=e.reset(seed=3200000000+i);assert not e.collision()
  assert all(not e.collision(np.r_[e.goal,a]) for a in np.linspace(-np.pi,np.pi,16,endpoint=False))
  # Validate actual rasterized boxes against static occupancy and every other box.
  res=e.grid.resolution
  for box in e.boxes_placed:
   lo=np.floor((np.array(box['center'])-np.array(box['size'])/2)/res).astype(int);hi=np.ceil((np.array(box['center'])+np.array(box['size'])/2)/res).astype(int)
   other=e.grid.cells.copy();other[lo[1]:hi[1],lo[0]:hi[0]]=e._base.cells[lo[1]:hi[1],lo[0]:hi[0]]
   distance=distance_transform_edt(other==0)*res-res*np.sqrt(2)
   assert distance[lo[1]:hi[1],lo[0]:hi[0]].min()>=.8
  rows.append(dict(variant=variant,seed=3200000000+i,boxes=len(e.boxes_placed),start=e.pose.tolist(),goal=e.goal.tolist()))
  if variant.endswith('treino') and i==0:
   fig=draw(e,out/'generated/arena_2016_treino_exemplo.png');plt.close(fig)
 e.close()
(out/'scenario_validation.json').write_text(json.dumps(dict(total=len(rows),passed=True,scenarios=rows),indent=2));print('validated',len(rows))
