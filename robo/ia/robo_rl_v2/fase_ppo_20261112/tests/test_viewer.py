import hashlib,json,shutil,tempfile,time,unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import torch
from robo_rl_v2.maps import ROOT
from robo_rl_v2.viewer_session import EvaluationSession,latest_checkpoint,load_policy
from robo_rl_v2.viewer import Viewer

torch.set_num_threads(1)

class FakeCanvas:
    def __init__(self):self.calls=[]
    def winfo_width(self):return 850
    def winfo_height(self):return 700
    def delete(self,*args):self.calls.append(('delete',args,{}))
    def __getattr__(self,name):
        if name.startswith('create_'):
            def record(*args,**kwargs):self.calls.append((name,args,kwargs));return len(self.calls)
            return record
        raise AttributeError(name)

class ViewerTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.run=Path(self.temp.name)
        (self.run/'checkpoints').mkdir()
        self.original=ROOT/'runs/smoke_final/checkpoints/ppo_000000001024.zip'
        self.path=self.run/'checkpoints/model.zip'
        shutil.copy2(self.original,self.path)
        self.publish(1024)
    def tearDown(self):self.temp.cleanup()
    def publish(self,steps):
        temp=self.run/'latest.tmp';temp.write_text(json.dumps(dict(model=str(self.path),steps=steps)));temp.replace(self.run/'latest.json')
    def test_manifest_only_complete_and_path_confinement(self):
        checkpoint=latest_checkpoint(self.run);self.assertEqual(checkpoint.steps,1024)
        (self.run/'checkpoints/pending.zip').write_bytes(b'incomplete')
        self.assertEqual(latest_checkpoint(self.run),checkpoint)
        self.path.write_bytes(b'incomplete');self.assertRaises(Exception,load_policy,latest_checkpoint(self.run))
        (self.run/'latest.json').write_text(json.dumps(dict(model=str(self.original),steps=1024)))
        self.assertRaises(ValueError,latest_checkpoint,self.run)
    def test_frozen_inference_restart_and_unique_metrics(self):
        checkpoint=latest_checkpoint(self.run);model=load_policy(checkpoint)
        self.assertFalse(model.policy.training)
        self.assertTrue(all(not p.requires_grad for p in model.policy.parameters()))
        before={k:v.clone() for k,v in model.policy.state_dict().items()}
        digest=hashlib.sha256(self.path.read_bytes()).hexdigest()
        session=EvaluationSession(max_steps=4,scenarios=2);session.activate(checkpoint,model)
        pose=session.env.pose.copy()
        while not session.done:session.step()
        self.assertEqual(session.summary()['episodes'],1)
        self.assertEqual(session.env.level,4)
        session.restart();np.testing.assert_equal(session.env.pose,pose)
        while not session.done:session.step()
        self.assertEqual(session.summary()['episodes'],1)
        session.next_episode()
        while not session.done:session.step()
        self.assertEqual(session.summary()['episodes'],2)
        for key,value in model.policy.state_dict().items():self.assertTrue(torch.equal(before[key],value))
        self.assertEqual(hashlib.sha256(self.path.read_bytes()).hexdigest(),digest)
    def test_hot_reload_and_partial_metrics_discard(self):
        session=EvaluationSession(max_steps=6)
        checkpoint=latest_checkpoint(self.run);session.activate(checkpoint,load_policy(checkpoint));session.step()
        viewer=Viewer.__new__(Viewer);viewer.session=session;viewer.args=SimpleNamespace(run=self.run,poll_seconds=1)
        viewer.loader=ThreadPoolExecutor(max_workers=1);viewer.future=None;viewer.pending=None;viewer.next_poll=0;viewer.status='';viewer.end_until=None;viewer.map_dirty=False
        try:
            # Publish a complete different checkpoint while this evaluation is mid-episode.
            alternate=ROOT/'runs/smoke_final/checkpoints/ppo_000000000256.zip'
            self.path=self.run/'checkpoints/new.zip';shutil.copy2(alternate,self.path);self.publish(256)
            viewer.poll_checkpoint(time.monotonic());self.assertIsNotNone(viewer.future)
            viewer.future.result(timeout=20);viewer.poll_checkpoint(time.monotonic())
            # Freeze the current policy for the entire episode; new weights wait at a boundary.
            self.assertEqual(session.checkpoint.steps,1024)
            self.assertIsNotNone(viewer.pending)
            while not session.done:session.step()
            viewer.end_until=time.monotonic()-1
            viewer.poll_checkpoint(time.monotonic())
            self.assertEqual(session.checkpoint.steps,256)
            self.assertEqual(session.env.steps,0)
            self.assertEqual(session.summary()['episodes'],0)
            self.assertEqual(session.history[-1]['steps'],1024)
            self.assertTrue(viewer.map_dirty)
        finally:viewer.loader.shutdown(wait=True)
    def test_canvas_scene_and_rotated_map_transform(self):
        session=EvaluationSession(max_steps=6);cp=latest_checkpoint(self.run);session.activate(cp,load_policy(cp));session.step()
        viewer=Viewer.__new__(Viewer);viewer.session=session;viewer.canvas=FakeCanvas();viewer.map_dirty=True
        with patch('PIL.ImageTk.PhotoImage',side_effect=lambda image:image):viewer.draw_episode()
        names=[call[0] for call in viewer.canvas.calls]
        self.assertIn('create_image',names);self.assertIn('create_polygon',names)
        self.assertGreater(names.count('create_line'),20)
        # Canvas rendering is tested headlessly; the same PIL map used by Tk is saved for inspection.
        viewer.map_image.save(ROOT/'docs/viewer/map_preview.png')
        xmin,ymin,xmax,ymax=viewer.bounds
        a=viewer.world_to_screen([[xmin,ymax],[xmax,ymin]])
        self.assertGreater(a[1,0],a[0,0]);self.assertGreater(a[1,1],a[0,1])
        # Nonzero YAML orientation must still preserve the world raster bounds and LiDAR projection.
        session.env.grid.origin[2]=.4
        c,s=np.cos(.4),np.sin(.4);session.env.grid.rotation=np.array([[c,-s],[s,c]])
        viewer.map_dirty=True
        with patch('PIL.ImageTk.PhotoImage',side_effect=lambda image:image):viewer.draw_episode()
        self.assertTrue(np.isfinite(viewer.world_to_screen(session.env.pose[:2])).all())

if __name__=='__main__':unittest.main()
