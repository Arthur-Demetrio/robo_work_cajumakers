"""Independent read-only hypotheses. Identity is tentative until verified."""
import math
from dataclasses import dataclass,field

def axis_error(a,b):
    return abs((a-b+math.pi/2)%math.pi-math.pi/2)

@dataclass
class Track:
    id:int
    x:float
    y:float
    yaw:float
    width:float
    n:int
    last_seen:float
    first_seen:float
    views:list=field(default_factory=list)
    side_hits:int=0
    side_depths:list=field(default_factory=list)
    ambiguous:bool=False
    @property
    def side_evidence(self):
        return self.side_hits>0

class Tracker:
    def __init__(self,center_gate=0.23,angle_gate_deg=22,max_age=12.0,
                 viewpoint_spacing=0.22,max_tracks=30,memory_age=180.0):
        self.center_gate=center_gate
        self.angle_gate=math.radians(angle_gate_deg)
        self.max_age=max_age
        self.memory_age=memory_age
        self.viewpoint_spacing=viewpoint_spacing
        self.max_tracks=max_tracks
        self.tracks=[]
        self.next_id=1
    def update(self,observations,sensor_pose,now):
        """obs: (x,y,axis,width,side_depths). Long memory only with side evidence."""
        self.tracks=[t for t in self.tracks if now-t.last_seen <= (self.memory_age if t.side_hits>=3 else self.max_age)]
        observations=[(*o, []) if len(o)==4 else o for o in observations]
        options=[]
        for oi,(x,y,a,w,depths) in enumerate(observations):
            for ti,t in enumerate(self.tracks):
                dist=math.hypot(x-t.x,y-t.y)
                angle=axis_error(a,t.yaw)
                if dist<=self.center_gate and angle<=self.angle_gate:
                    options.append((dist/self.center_gate+angle/self.angle_gate+abs(w-t.width),oi,ti))
        used_o=set();used_t=set()
        for _,oi,ti in sorted(options):
            if oi in used_o or ti in used_t:continue
            used_o.add(oi);used_t.add(ti)
            x,y,a,w,depths=observations[oi]
            t=self.tracks[ti]
            t.x=.8*t.x+.2*x;t.y=.8*t.y+.2*y
            t.width=.8*t.width+.2*w
            delta=(a-t.yaw+math.pi/2)%math.pi-math.pi/2
            t.yaw=(t.yaw+.2*delta)%math.pi
            t.n+=1;t.last_seen=now
            if len(t.views)<24 and all(math.hypot(sensor_pose[0]-v[0],sensor_pose[1]-v[1])>=self.viewpoint_spacing for v in t.views):
                t.views.append(tuple(sensor_pose))
            if depths:
                t.side_hits+=1
                t.side_depths.extend(depths)
                t.side_depths=t.side_depths[-30:]
        for oi,(x,y,a,w,depths) in enumerate(observations):
            if oi in used_o or len(self.tracks)>=self.max_tracks:continue
            t=Track(self.next_id,x,y,a,w,1,now,now,[tuple(sensor_pose)])
            if depths:t.side_hits=1;t.side_depths=list(depths)
            self.next_id+=1
            self.tracks.append(t)
        return self.tracks
