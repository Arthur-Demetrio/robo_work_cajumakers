"""Conservative, ROS-independent recovery decisions. Unknown space is forbidden.

The coverage model is a 2-D sampled swept volume, not a physical safety proof.
A calibrated self-shadow mask and an independently measured pose are prerequisites.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Motion:
    name: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Scan:
    ranges: tuple
    angle_min: float
    increment: float
    range_min: float
    range_max: float
    laser: tuple  # calibrated TF: x, y, yaw in base_link
    shadows: tuple = ()  # closed intervals in laser frame, radians
    certified: bool = False


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def inside(x, y, footprint, margin=0.0):
    rear, front, half = footprint
    return rear - margin <= x <= front + margin and abs(y) <= half + margin


def covered(scan, x, y, resolution=0.01):
    if (not scan.certified or scan.increment <= 0 or not scan.ranges or
        not all(math.isfinite(v) for v in (*scan.laser, scan.angle_min, scan.increment,
                                         scan.range_min, scan.range_max)) or
        any(not math.isfinite(lo) or not math.isfinite(hi) or not -math.pi <= lo <= hi <= math.pi
            for lo, hi in scan.shadows)):
        return False
    lx, ly, yaw = scan.laser
    dx, dy = x - lx, y - ly
    r, a = math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw)
    if r < scan.range_min + resolution or r > scan.range_max - resolution:
        return False
    # Normalize the sample angle to the scan's actual interval, without assuming 360 degrees.
    while a < scan.angle_min:
        a += 2 * math.pi
    while a > scan.angle_min + 2 * math.pi:
        a -= 2 * math.pi
    for lo, hi in scan.shadows:
        if lo <= wrap(a) <= hi:
            return False
    idx = int(round((a - scan.angle_min) / scan.increment))
    # Require neighboring rays; an isolated rear return proves nothing.
    full = len(scan.ranges) * scan.increment >= 2 * math.pi - scan.increment * 1.5
    if not full and (idx <= 0 or idx >= len(scan.ranges) - 1):
        return False
    if full:
        idx %= len(scan.ranges)
    if r * scan.increment > resolution:
        return False  # angular holes too large for this grid
    return all(math.isfinite(scan.ranges[j % len(scan.ranges)]) and
               scan.range_min <= scan.ranges[j % len(scan.ranges)] <= scan.range_max and
               scan.ranges[j % len(scan.ranges)] > r + resolution
               for j in (idx - 1, idx, idx + 1))


def swept_clear(scan, motion, footprint, margin=0.04, grid=0.02):
    """Every newly swept cell must have finite, unoccluded line-of-sight evidence.

    Inflate grid cells and interpolation by one grid width. Never discard a single
    obstacle as noise. Existing footprint interior is excluded from free-space tests.
    """
    if not scan.certified:
        return False
    rear, front, half = footprint
    radius = math.hypot(max(abs(rear), abs(front)), half)
    steps = max(1, math.ceil((math.hypot(motion.x, motion.y) +
                              radius * abs(motion.yaw)) / grid))
    # Obstacles in ANY swept footprint, including initial pose, veto the motion.
    lx, ly, lyaw = scan.laser
    obstacles = []
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            a = scan.angle_min + i * scan.increment + lyaw
            x, y = lx + r * math.cos(a), ly + r * math.sin(a)
            if not inside(x, y, footprint):
                obstacles.append((x, y))
    pad = margin + grid
    nx = math.ceil((front - rear + 2 * pad) / grid)
    ny = math.ceil((2 * half + 2 * pad) / grid)
    for k in range(steps + 1):
        f = k / steps
        cx, cy, a = motion.x * f, motion.y * f, motion.yaw * f
        c, s = math.cos(a), math.sin(a)
        for x, y in obstacles:
            if inside(c * (x - cx) + s * (y - cy),
                      -s * (x - cx) + c * (y - cy), footprint, pad):
                return False
        for ix in range(nx + 1):
            x = rear - pad + ix * (front - rear + 2 * pad) / nx
            for iy in range(ny + 1):
                y = -half - pad + iy * (2 * half + 2 * pad) / ny
                bx, by = cx + c * x - s * y, cy + s * x + c * y
                if not inside(bx, by, footprint) and not covered(scan, bx, by, grid):
                    return False
    return True


def choose(scan, footprint, heading, distance=0.10, backed=False, need_turn=False):
    """Prefer lateral + viable continuation, then short rear, then necessary turn.

    Post-rear decisions always consume a NEW scan in the server. Never simulate a
    rear translation of old obstacle points to authorize a subsequent turn.
    """
    if heading is None or not math.isfinite(heading):
        return None
    sides = (1, -1) if heading >= 0 else (-1, 1)
    for side in sides:
        lateral = Motion('left' if side > 0 else 'right', y=side * distance)
        # Check a continuation in the path direction, not just an empty lateral sector.
        continuation = Motion('continuation', x=0.08 * math.cos(heading),
                              y=lateral.y + 0.08 * math.sin(heading))
        if swept_clear(scan, lateral, footprint) and swept_clear(scan, continuation, footprint):
            return lateral
    if not backed:
        rear = Motion('rear', x=-min(distance, 0.10))
        if swept_clear(scan, rear, footprint):
            return rear
    if need_turn and abs(heading) > math.radians(45):
        turn = Motion('turn', yaw=max(-0.70, min(0.70, heading)))
        if swept_clear(scan, turn, footprint):
            return turn
    return None


class Session:
    """Latched safety failure survives action retries and new goals."""
    def __init__(self):
        self.state = 'IDLE'
        self.reason = ''
        self.attempts = 0
        self.active = False

    def begin(self):
        if self.active or self.state == 'FAILED' or self.attempts >= 2:
            return False
        self.active = True
        self.attempts += 1
        self.state = 'STOP'
        return True

    def fail(self, reason):
        self.state, self.reason, self.active = 'FAILED', reason, False

    def finish(self):
        if self.state == 'FAILED':
            return False
        self.state, self.active = 'IDLE', False
        return True

    def authorize(self):
        if self.active:
            return False
        self.state, self.reason, self.attempts = 'IDLE', '', 0
        return True


def measured_progress(start, end, motion):
    """Independent fixed-frame base pose, not odometry or a velocity integral."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    bx, by = c * dx + s * dy, -s * dx + c * dy
    if motion.yaw:
        return wrap(end[2] - start[2]) * math.copysign(1, motion.yaw)
    length = math.hypot(motion.x, motion.y)
    return (bx * motion.x + by * motion.y) / length if length else 0.0


def remaining_path_length(points, position):
    """Remaining directed polyline arc plus distance to it; no odometry input."""
    if not points:
        return None
    if len(points) == 1:
        return math.dist(points[0], position)
    segments = [math.dist(a, b) for a, b in zip(points, points[1:])]
    total, prefix, best = sum(segments), 0.0, None
    for (a, b), length in zip(zip(points, points[1:]), segments):
        if length <= 1e-9:
            continue
        dx, dy = b[0] - a[0], b[1] - a[1]
        f = max(0.0, min(1.0, ((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(length*length)))
        distance = math.hypot(position[0]-a[0]-f*dx, position[1]-a[1]-f*dy)
        arc = prefix + f*length
        key = (distance, arc)  # conservative earliest segment at intersections
        if best is None or key < best[0]:
            best = key, total-arc+distance
        prefix += length
    return best[1] if best else math.dist(points[-1], position)
