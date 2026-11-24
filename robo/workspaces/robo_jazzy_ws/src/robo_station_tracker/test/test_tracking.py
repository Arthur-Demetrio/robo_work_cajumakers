import math
from robo_station_tracker.tracking import Tracker, axis_error

def test_axis_orientation():
    assert axis_error(0,math.pi)<1e-6

def test_reobservation_two_targets():
    t=Tracker()
    t.update([(1,1,0,0.88),(3,2,0,0.92)],(0,0),1.0)
    assert [x.id for x in t.tracks]==[1,2]
    t.update([(1.03,0.99,0.02,0.89)],(0.3,0),2.0)
    assert len(t.tracks)==2 and t.tracks[0].n==2 and len(t.tracks[0].views)==2

def test_never_confirm_from_repetition():
    t=Tracker()
    for i in range(100):
        t.update([(1,1,0,0.9)],(0.0,0.0),i/10)
    assert len(t.tracks)==1 and not t.tracks[0].side_evidence

def test_lost_and_return():
    t=Tracker(max_age=12)
    t.update([(1,1,0,0.9)],(0,0),1)
    t.update([], (0,0),2)
    t.update([(1.01,1.02,0,0.9)],(0,0),4)
    assert len(t.tracks)==1 and t.tracks[0].n==2
