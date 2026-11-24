import math


def extract_segments(points, max_gap=0.13, min_points=6, max_residual=0.035):
    """Split scan-ordered xy points into contiguous straight-line candidates."""
    clusters = []
    current = []
    for point in points:
        if point is None:
            if current:
                clusters.append(current)
                current = []
            continue
        if current and math.dist(point, current[-1]) > max_gap:
            clusters.append(current)
            current = []
        current.append(point)
    if current:
        clusters.append(current)
    output = []
    for cluster in clusters:
        if len(cluster) < min_points:
            continue
        output.extend(_recursive_split(cluster, min_points, max_residual))
    return output


def _recursive_split(points, min_points, threshold):
    if len(points) < min_points:
        return []
    p0, p1 = points[0], points[-1]
    length = math.dist(p0, p1)
    if length < 1e-5:
        return []
    residuals = [abs((p1[0]-p0[0])*(p0[1]-p[1]) - (p0[0]-p[0])*(p1[1]-p0[1]))/length for p in points]
    largest = max(range(len(points)), key=lambda i: residuals[i])
    if residuals[largest] > threshold and largest >= min_points-1 and len(points)-largest >= min_points:
        return _recursive_split(points[:largest+1], min_points, threshold)+_recursive_split(points[largest:], min_points, threshold)
    if max(residuals) > threshold:
        return []
    return [(p0, p1, len(points), max(residuals))]


def select_candidate(segments, target_width=0.90, width_tolerance=0.15, max_distance=3.0, max_bearing=1.1):
    valid=[]
    for start, end, n, residual in segments:
        width=math.dist(start,end)
        cx=(start[0]+end[0])/2
        cy=(start[1]+end[1])/2
        distance=math.hypot(cx,cy)
        bearing=abs(math.atan2(cy,cx))
        if abs(width-target_width)>width_tolerance or distance>max_distance or cx <= 0.0 or bearing>max_bearing:
            continue
        # Long narrow structures may be walls. This is only a candidate, never classified definitively.
        score=abs(width-target_width)/width_tolerance+0.18*distance+0.3*bearing+2*residual
        valid.append((score,start,end,width,n))
    return min(valid,key=lambda item:item[0]) if valid else None


def approach_pose(start, end, clearance=0.20, front_offset=0.365):
    """Return (x,y,yaw) relative to laser frame, plus segment midpoint."""
    cx=(start[0]+end[0])/2
    cy=(start[1]+end[1])/2
    tx=end[0]-start[0]
    ty=end[1]-start[1]
    length=math.hypot(tx,ty)
    if length<1e-6:
        raise ValueError('degenerate segment')
    nx=-ty/length
    ny=tx/length
    # Facing sensor side: laser origin is at (0,0)
    if nx*cx+ny*cy>0:
        nx=-nx
        ny=-ny
    offset=front_offset+clearance
    ax=cx+nx*offset
    ay=cy+ny*offset
    yaw=math.atan2(-ny,-nx)
    return ax,ay,yaw,cx,cy
