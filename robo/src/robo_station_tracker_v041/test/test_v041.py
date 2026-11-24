import math
import unittest

from robo_station_tracker_v041.detection import adaptive_gap,extract_segments
from robo_station_tracker_v041.tracking import Observation,Tracker,geometry_match
from robo_station_tracker_v041.geometry import side_evidence,continuation
from robo_station_tracker_v041.tracking import Track
from robo_station_tracker_v041.rectangle import rectangle_hypothesis


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
        # Duas faces completas com pouca sobreposicao permanecem diferentes.
        self.assertEqual(len(t.tracks),2)
        self.assertLessEqual(max(x.width for x in t.tracks),1.05)

    def test_partial_does_not_grow_face(self):
        t=Tracker()
        for i in range(30):
            t.update([obs(0.,.90)],(0.,-1.),i*.1)
        baseline=t.tracks[0].width
        for i in range(30,60):
            t.update([obs(.30,.88)],(.35,-1.),i*.1)
        self.assertAlmostEqual(t.tracks[0].width,baseline,delta=.012)

    def test_tolerate_adjacent_wall_behind(self):
        t=Tracker()
        t.update([obs(0.,.90,0),obs(0.,.90,.30)],(0,-1),0.)
        self.assertEqual(len(t.tracks),2)
        for i in range(10):
            t.update([obs(.06,.86,0),obs(.04,.88,.30)],(.3,-1),i*.1+1)
        self.assertEqual(len(t.tracks),2)
        for item in t.tracks:self.assertLess(abs(item.center[1]-.15),.151)

    def test_two_sides_user_snapshot(self):
        def make(i, center,width,axis,now):
            dx=math.cos(axis)*width*.5;dy=math.sin(axis)*width*.5
            return Track(i,(center[0]-dx,center[1]-dy),(center[0]+dx,center[1]+dy),15,now,0)
        front=make(35,(2.768,4.215),1.059,1.634,200)
        lower=make(34,(2.998,3.715),.520,2.980,80.41)
        upper=make(41,(2.985,4.647),.499,.018,99.94)
        r=rectangle_hypothesis(front,[front,lower,upper],200)
        self.assertIsNotNone(r)
        self.assertEqual(r['tracks_laterais'],['TRACK_34','TRACK_41'])
        self.assertAlmostEqual(r['largura_entre_laterais_m'],.884,delta=.03)
        self.assertTrue('NAO_VERIFICADA' in r['tipo'])

    def test_opposite_side_directions_rejected(self):
        front=Track(1,(-.45,0),(.45,0),3,1,0)
        up=Track(2,(-.45,0),(-.45,.35),3,1,0)
        down=Track(3,(.45,0),(.45,-.35),3,1,0)
        self.assertIsNone(rectangle_hypothesis(front,[front,up,down],1.))

    def test_reset(self):
        t=Tracker();t.update([obs(0,.86)],(0,-1),1.)
        t.reset();self.assertEqual(len(t.tracks),0)

if __name__=='__main__':unittest.main()
