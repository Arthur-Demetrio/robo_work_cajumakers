"""Conservative right-angle / depth evidence from a *single* laser scan."""
import math
from .tracking import axis_error

def side_evidence(front, segments, depth_min=0.25, depth_max=0.50,
                  corner_tol=0.10, angle_tol_deg=22):
    """Return observed side depths at the endpoints of front (possibly empty).

    A perpendicular must meet a physical endpoint, not cross the middle of a
    long wall. The side must extend away from the sensor (toward the box).
    It is evidence, not proof of object identity; junctions can mimic corners.
    """
    a,b,_,_=front
    ux,uy=b[0]-a[0],b[1]-a[1]
    size=math.hypot(ux,uy)
    if size<1e-6:return []
    nx,ny=-uy/size,ux/size
    mid=((a[0]+b[0])/2,(a[1]+b[1])/2)
    # Normal pointing *away* from the LiDAR origin.
    if nx*mid[0]+ny*mid[1]<0:nx,ny=-nx,-ny
    hits=[]
    for idx,corner in enumerate((a,b)):
        best=None
        for other in segments:
            if other is front:continue
            c,d,_,_=other
            vx,vy=d[0]-c[0],d[1]-c[1]
            length=math.hypot(vx,vy)
            if not (depth_min <= length <= depth_max):continue
            if abs(math.pi/2-axis_error(math.atan2(uy,ux),math.atan2(vy,vx)))>math.radians(angle_tol_deg):continue
            if math.dist(c,corner)<=corner_tol: start,end=c,d
            elif math.dist(d,corner)<=corner_tol:start,end=d,c
            else:continue
            displacement=((end[0]-start[0])*nx+(end[1]-start[1])*ny)
            if displacement < depth_min*0.65:continue
            if best is None or length>best:best=length
        if best is not None:hits.append((idx,best))
    return hits
