import tempfile,unittest,struct
from pathlib import Path
import numpy as np
from PIL import Image
from stable_baselines3.common.env_checker import check_env
from gymnasium.utils.env_checker import check_env as gym_check
from robo_rl_v2.env import RoboEnv,FOOTPRINT,rotation
from robo_rl_v2.maps import Grid,ROOT,inventory
from robo_rl_v2.events import ScalarEvents,crc32c,masked_crc

class TestMap(unittest.TestCase):
    def test_origin_rotation_and_thresholds(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp);Image.fromarray(np.array([[0,205],[254,255]],np.uint8)).save(path/'map.pgm')
            (path/'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [2, 3, 1.5707963267948966]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            grid=Grid.load(path/'map.yaml')
            np.testing.assert_equal(grid.cells,[[0,0],[1,-1]])
            np.testing.assert_allclose(grid.world([.1,.2]),[1.8,3.1])
            np.testing.assert_allclose(grid.local([1.8,3.1]),[.1,.2])
            self.assertTrue(grid.hits(grid.world([.15,.15])))
            self.assertTrue(grid.hits(grid.world([-.01,0])))
    def test_real_assets(self):
        rows=inventory(ROOT/'assets/maps');self.assertEqual(len(rows),2)
        self.assertTrue(all(r['free']>0 and r['occupied']>0 and r['unknown']>0 for r in rows))

class TestEnv(unittest.TestCase):
    def test_api(self):
        env=RoboEnv(max_steps=3);check_env(env);gym_check(env,skip_render_check=True)
    def test_all_levels_deterministic_and_connected(self):
        for level in range(1,6):
            env=RoboEnv(level,max_steps=3)
            for seed in range(5):
                obs,_=env.reset(seed=seed);pose=env.pose.copy();cells=env.grid.cells.copy()
                self.assertFalse(env.collision())
                self.assertTrue(env.observation_space.contains(obs))
                out=env.step(np.array([.5,.2,.3]))
                np.testing.assert_equal(obs,env.reset(seed=seed)[0]);np.testing.assert_equal(env.pose,pose)
                np.testing.assert_equal(env.grid.cells,cells)
                np.testing.assert_equal(out[0],env.step(np.array([.5,.2,.3]))[0])
    def test_footprint_interior_cell_and_rotation(self):
        env=RoboEnv();env.reset(seed=4)
        grid=Grid(np.zeros((100,100),np.int8),.05);env.grid=grid;env.pose=np.array([2.,2.,0.])
        grid.cells[40,40]=1;grid.refresh();self.assertTrue(env.collision())
        grid.cells[:]=0;grid.cells[40,46]=1;grid.refresh()
        self.assertTrue(env.collision());self.assertFalse(env.collision([2.,2.,np.pi]))
    def test_swept_high_speed_thin_wall(self):
        env=RoboEnv(max_steps=4);env.reset(seed=1)
        cells=np.zeros((100,100),np.int8);cells[:,45]=1
        env.grid=Grid(cells,.05);env.pose=np.array([1.5,2.,0.]);env.estimate=env.pose.copy();env.goal=np.array([4.,2.])
        env.velocity[:]=[10,0,0];env.decel[:]=0
        _,_,term,_,info=env.step([1,0,0]);self.assertTrue(term);self.assertTrue(info['collision']);self.assertFalse(env.collision())
        self.assertLess(env.pose[0]+.365,2.25)
    def test_rotational_sweep(self):
        env=RoboEnv();env.reset(seed=1);cells=np.zeros((100,100),np.int8)
        env.grid=Grid(cells,.05);env.pose=np.array([2.,2.,0.]);env.estimate=env.pose.copy()
        cells[48,40]=1;env.grid.refresh()
        self.assertFalse(env.collision());env.velocity[:]=[0,0,20];env.decel[:]=0
        _,_,_,_,info=env.step([0,0,1]);self.assertTrue(info['collision'])
    def test_odometry_cannot_reward_or_change_true_metrics(self):
        a=RoboEnv();b=RoboEnv();a.reset(seed=8);b.reset(seed=8)
        b.estimate+=100
        x=a.step([0,0,0]);y=b.step([0,0,0])
        self.assertEqual(x[1],y[1]);self.assertEqual(x[4],y[4]);self.assertLess(x[1],0)
        self.assertFalse(np.array_equal(x[0],y[0]))
    def test_delay_acceleration_and_mask(self):
        env=RoboEnv();env.reset(seed=1);env.step([1,0,0])
        self.assertLessEqual(env.velocity[0],env.accel[0]*env.dt)
        self.assertTrue(np.all(env.scan[0][env.scan[1]==0]==0));self.assertGreater(np.count_nonzero(env.scan[1]==0),0)
    def test_truncation_and_reset_required(self):
        env=RoboEnv(max_steps=1);env.reset(seed=1)
        _,_,term,trunc,_=env.step([0,0,0]);self.assertTrue(trunc);self.assertFalse(term)
        with self.assertRaises(RuntimeError):env.step([0,0,0])
    def test_curriculum(self):
        env=RoboEnv();env.set_curriculum([10,20,30,40]);env.transitions=35;env.reset(seed=3);self.assertEqual(env.level,4)

class TestEvents(unittest.TestCase):
    def test_crc_and_record(self):
        self.assertEqual(crc32c(b'123456789'),0xe3069283)
        with tempfile.TemporaryDirectory() as tmp:
            events=ScalarEvents(tmp);events.scalar('success',.5,25);events.close()
            data=next(Path(tmp).iterdir()).read_bytes();offset=0;count=0
            while offset<len(data):
                size=data[offset:offset+8];n=struct.unpack('<Q',size)[0]
                self.assertEqual(data[offset+8:offset+12],masked_crc(size))
                body=data[offset+12:offset+12+n]
                self.assertEqual(data[offset+12+n:offset+16+n],masked_crc(body));offset+=16+n;count+=1
            self.assertEqual(count,2)

if __name__=='__main__':unittest.main()
