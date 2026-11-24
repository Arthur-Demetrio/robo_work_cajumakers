from robo_station_tracker.geometry import side_evidence

def test_rectangle_side():
    front=((1,-.45),(1,.45),20,0.0)
    side=((1,.45),(1.35,.45),10,0.0)
    assert len(side_evidence(front,[front,side]))==1

def test_wall_split_is_not_side():
    front=((1,-.45),(1,.45),20,0.0)
    continuation=((1,.45),(1,1.2),10,0.0)
    assert side_evidence(front,[front,continuation])==[]

def test_long_side_rejected():
    front=((1,-.45),(1,.45),20,0.0)
    other=((1,.45),(1.8,.45),15,0.0)
    assert side_evidence(front,[front,other])==[]
