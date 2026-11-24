import math
import unittest

from robo_station_tracker_v04.detection import adaptive_gap,extract_segments
from robo_station_tracker_v04.tracking import Observation,Tracker,geometry_match
from robo_station_tracker_v04.geometry import side_evidence,continuation


def obs(x0,x1,y=0,depths=None,wall=False):
    return Observation((x0,y),(x1,y),depths or [],wall)

class TestDetector(unittest.TestCase):
    def test_gap_grows_with_distance(self):
        near=adaptive_gap((1.,0.),(1.,.05),math.radians(2))
        far=adaptive_gap((4.,0.),(4.,.14),math.radians(2))
        self.assertGreater(far,near)
        self.assertLessEqual(far,.34)

    def test_gap_adaptive_vs_fixed(self):
        # feixes na reta x=4m, espacamento >13cm e <limiar adaptativo
        pts=[(4.,i*.14) for i in range(9)]
        self.assertEqual(len(extract_segments(pts,math.radians(2),adaptive=False)),0)
        self.assertEqual(len(extract_segments(pts,math.radians(2),adaptive=True)),1)

    def test_partial_track_same_identity_after_occlusion(self):
        t=Tracker(short_age=12.,view_spacing=.23)
        t.update([obs(0.,.86)],(0.,-1.),0.)
        tid=t.tracks[0].id
        t.update([obs(.22,.87)],(.38,-1.),1.)
        self.assertEqual(len(t.tracks),1)
        self.assertEqual(t.tracks[0].id,tid)
        self.assertEqual(len(t.tracks[0].views),2)
        t.update([],(.38,-1.),4.)
        t.update([obs(.04,.79)],(.50,-1.),8.)
        self.assertEqual(len(t.tracks),1)
        self.assertEqual(t.tracks[0].id,tid)

    def test_nearby_parallel_wall_separate(self):
        t=Tracker()
        t.update([obs(0.,.9,0.),obs(0.,.9,.30)],(0.,-1.),1.)
        self.assertEqual(len(t.tracks),2)

    def test_cannot_promote_wall_by_repetition(self):
        t=Tracker()
        for i in range(100):
            t.update([obs(0,.9,0,[],True)],(0,-1),i*.1)
        self.assertTrue(t.tracks[0].suspected_wall)
        self.assertFalse('GEOMETRIA_COMPATIVEL' in t.status(t.tracks[0],10.0))

    def test_geometric_only_with_distinct_depth_views(self):
        t=Tracker(view_spacing=.2)
        for i in range(10):t.update([obs(0,.88,0,[.31])],(0.,-1.),float(i)*.1)
        self.assertEqual(len(t.tracks[0].depth_views),1)
        self.assertNotIn('GEOMETRIA_COMPATIVEL',t.status(t.tracks[0],1.))
        for i in range(10,20):t.update([obs(.02,.89,0,[.31])],(.35,-1.),float(i)*.1)
        self.assertEqual(len(t.tracks[0].depth_views),2)
        self.assertIn('GEOMETRIA_COMPATIVEL',t.status(t.tracks[0],2.))

    def test_wall_continuation(self):
        front=((0.,0.),(.88,0.),8,.0)
        wall=((.90,0.),(1.60,0.),9,.0)
        self.assertTrue(continuation(front,[front,wall]))
        shifted=((.90,.32),(1.60,.32),9,.0)
        self.assertFalse(continuation(front,[front,shifted]))

    def test_side_evidence(self):
        front=((-.45,1.),(.45,1.),18,.0)
        side=((-.45,1.),(-.45,1.31),8,.0)
        # LiDAR esta em (0,0), normal para o lado +y
        self.assertTrue(side_evidence(front,[front,side],.25,.50,.10))

    def test_long_wall_not_force_merge(self):
        t=Tracker()
        t.update([obs(0,.89)],(0,-1.),0.)
        t.update([obs(.65,1.60)],(.3,-1.),1.)
        # linha grande e extensao excessiva sinalizam suspeita;
        # deteccao isolada ainda pode iniciar outra track, sem confirmar.
        self.assertTrue(any(a.suspected_wall for a in t.tracks))

    def test_reset(self):
        t=Tracker();t.update([obs(0,.86)],(0,-1),1.)
        t.reset();self.assertEqual(len(t.tracks),0)

if __name__=='__main__':unittest.main()
