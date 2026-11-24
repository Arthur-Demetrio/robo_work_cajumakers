import math
from robo_station_detector.detection import extract_segments,select_candidate,approach_pose

def test_segment():
    pts=[(1.0,y/100.0) for y in range(-45,46,3)]
    segments=extract_segments(pts)
    chosen=select_candidate(segments)
    assert chosen is not None
    assert abs(chosen[3]-0.90)<0.02

def test_approach():
    x,y,yaw,cx,cy=approach_pose((1,-.45),(1,.45))
    assert math.isclose(x,.435,abs_tol=.001)
    assert math.isclose(y,0,abs_tol=.001)
    assert math.isclose(yaw,0,abs_tol=.001)
