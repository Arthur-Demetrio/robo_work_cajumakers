import sys,time,cProfile,pstats
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.env import RoboEnv
e=RoboEnv();e.reset(seed=1)
p=cProfile.Profile();p.enable()
for i in range(100):e.step([0,0,0])
p.disable();pstats.Stats(p).sort_stats('cumtime').print_stats(15)
