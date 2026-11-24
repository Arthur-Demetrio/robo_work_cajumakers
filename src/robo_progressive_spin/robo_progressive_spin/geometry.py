"""Collision geometry for a rectangular mecanum footprint on an OccupancyGrid.

All coordinates are in metres. Unknown cells are considered unsafe. No dependency
on ROS modules so this can be tested without a robot.
"""
import math


def yaw_of_quat(q):
    # x/y/z/w quaternion
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def inside_polygon(x, y, vertices):
    inside = False
    j = len(vertices) - 1
    for i, (xi, yi) in enumerate(vertices):
        xj, yj = vertices[j]
        if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def to_world(xr, yr, pose):
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    return x + c * xr - s * yr, y + s * xr + c * yr


def grid_coords(grid, wx, wy):
    orig = grid.info.origin
    ox, oy = orig.position.x, orig.position.y
    phi = yaw_of_quat(orig.orientation)
    dx, dy = wx - ox, wy - oy
    c, s = math.cos(phi), math.sin(phi)
    xg = c * dx + s * dy
    yg = -s * dx + c * dy
    r = grid.info.resolution
    return math.floor(xg / r), math.floor(yg / r)


def grid_blocked(grid, wx, wy, threshold=100):
    ix, iy = grid_coords(grid, wx, wy)
    if ix < 0 or iy < 0 or ix >= grid.info.width or iy >= grid.info.height:
        return True
    cost = grid.data[iy * grid.info.width + ix]
    return cost < 0 or cost >= threshold


def footprint_clear(grid, pose, footprint, threshold=100):
    """Scan occupied grid cell centres within polygon (and each polygon vertex).

    OccupancyGrid lethal obstacle value 100 and unknown -1 are blocked;
    inflated cells 1..99 are not treated as hard obstacles. Uses conservative
    unknown-as-blocked semantics. Footprint must already contain safety margin.
    """
    world_vertices = [to_world(px, py, pose) for px, py in footprint]
    if any(grid_blocked(grid, x, y, threshold) for x, y in world_vertices):
        return False
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    phi = yaw_of_quat(grid.info.origin.orientation)
    c, s = math.cos(phi), math.sin(phi)
    resolution = grid.info.resolution
    coords = [grid_coords(grid, x, y) for x, y in world_vertices]
    minx, maxx = min(p[0] for p in coords)-1, max(p[0] for p in coords)+1
    miny, maxy = min(p[1] for p in coords)-1, max(p[1] for p in coords)+1
    if minx < 0 or miny < 0 or maxx >= grid.info.width or maxy >= grid.info.height:
        return False
    for iy in range(miny, maxy+1):
        gy = (iy + 0.5) * resolution
        for ix in range(minx, maxx+1):
            gx = (ix + 0.5) * resolution
            wx = ox + c * gx - s * gy
            wy = oy + s * gx + c * gy
            if inside_polygon(wx, wy, world_vertices):
                v = grid.data[iy * grid.info.width+ix]
                if v < 0 or v >= threshold:
                    return False
    return True


def arc_clear(grid, pose, footprint, delta_yaw, sample_rad=0.02):
    count = max(1, int(math.ceil(abs(delta_yaw) / sample_rad)))
    for i in range(count + 1):
        angle = pose[2] + delta_yaw * i / count
        if not footprint_clear(grid, (pose[0], pose[1], angle), footprint):
            return False
    return True


def movement_clear(grid, pose, footprint, dx, dy, sample_m=0.02):
    n = max(1, int(math.ceil(math.hypot(dx, dy) / sample_m)))
    for i in range(n + 1):
        f = i / n
        x, y = to_world(dx * f, dy * f, pose)
        if not footprint_clear(grid, (x, y, pose[2]), footprint):
            return False
    return True


def choose_lateral_escape(left_distance, right_distance, half_width=0.315,
                          margin=0.03, desired=0.10, minimum=0.04,
                          measurement_reserve=0.01, quantum=0.01):
    """Return signed lateral target in base frame or None if neither side fits.

    Distances must be *measured from base_link along each side*, in metres.
    This is an advisory selection, not a substitute for swept-path collision
    checking, sonar/ToF or a collision monitor.
    """
    candidates = []
    for side, distance, sign in (("left", left_distance, 1),
                                 ("right", right_distance, -1)):
        free = distance - half_width - margin - measurement_reserve
        if math.isfinite(free):
            amount = math.floor((min(desired, free) + 1e-8)/quantum) * quantum
            if amount + 1e-8 >= minimum:
                candidates.append((amount, side, sign))
    if not candidates:
        return None
    # Larger safe motion has priority; on tie, right for deterministic logs.
    amount, side, sign = max(candidates, key=lambda item:(item[0], item[2] < 0))
    return {"side": side, "distance": round(amount, 3),
            "signed_y": round(sign * amount, 3)}


# ---------------------------------------------------------------------------
# Folga continua durante o giro (correcao 2026-10-20).
# As metricas antigas (arco limitado ao horizonte e folga em degraus de 1 cm
# ate 4 cm) saturavam nos dois sentidos e o desempate caia sempre no sentido
# pedido pela BT (+1). Esta metrica mede, sem teto baixo, a menor distancia
# entre o contorno do robo e as celulas letais ao fim do micro-giro.
# ---------------------------------------------------------------------------
def obstacle_points(grid, pose, radius=1.0, threshold=100):
    """Centros das celulas letais (>= threshold) ate `radius` do robo."""
    info = grid.info
    r = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    phi = yaw_of_quat(info.origin.orientation)
    c, s = math.cos(phi), math.sin(phi)
    cx, cy = grid_coords(grid, pose[0], pose[1])
    k = int(math.ceil(radius / r))
    points = []
    for iy in range(max(0, cy - k), min(info.height, cy + k + 1)):
        for ix in range(max(0, cx - k), min(info.width, cx + k + 1)):
            if grid.data[iy * info.width + ix] >= threshold:
                gx, gy = (ix + 0.5) * r, (iy + 0.5) * r
                wx, wy = ox + c * gx - s * gy, oy + s * gx + c * gy
                if math.hypot(wx - pose[0], wy - pose[1]) <= radius:
                    points.append((wx, wy))
    return points


def _segment_distance(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    length2 = vx * vx + vy * vy
    t = 0.0 if length2 <= 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / length2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def footprint_distance(points, pose, footprint, cap=0.30):
    """Menor distancia entre os pontos e o contorno do footprint na pose (0 se dentro)."""
    world = [to_world(x, y, pose) for x, y in footprint]
    best = cap
    for px, py in points:
        if inside_polygon(px, py, world):
            return 0.0
        for i in range(len(world)):
            ax, ay = world[i]
            bx, by = world[(i + 1) % len(world)]
            best = min(best, _segment_distance(px, py, ax, ay, bx, by))
    return best


def spin_options(grid, pose, footprint, amount, horizon, opening_distance):
    """Evaluate both swept arcs, stopping each at the first unsafe pose.

    Scores are geometric, independent of inflation costs. Unknown is blocked.
    Returns only directions whose entire first micro-step is clear.
    """
    horizon = max(amount, min(horizon, 0.70))
    points = obstacle_points(grid, pose)
    options = []
    for sign in (1, -1):
        if not arc_clear(grid, pose, footprint, sign * amount):
            continue
        safe_angle = amount
        best_opening = 0.0
        first_opening = None
        # Prefix testing: never score an opening beyond a blocked arc.
        n = max(1, math.ceil(horizon / 0.02))
        for i in range(1, n + 1):
            angle = horizon * i / n
            trial = (pose[0], pose[1], pose[2] + sign * angle)
            if not footprint_clear(grid, trial, footprint):
                break
            safe_angle = max(safe_angle, angle)
            if i == n or i % 3 == 0:
                for distance in sorted(set((0.08, 0.14, opening_distance))):
                    if distance > opening_distance:
                        continue
                    if not movement_clear(grid, trial, footprint, distance, 0):
                        break
                    best_opening = max(best_opening, distance)
                    if distance >= opening_distance and first_opening is None:
                        first_opening = angle
        # Extra clearance of the *whole immediate arc*, not just its endpoint.
        clearance = 0.0
        for pad in (0.01, 0.02, 0.03, 0.04):
            expanded = [(x + math.copysign(pad, x),
                         y + math.copysign(pad, y)) for x, y in footprint]
            if not arc_clear(grid, pose, expanded, sign * amount):
                break
            clearance = pad
        # Folga continua ao fim do micro-giro e de dois micro-giros (se livres).
        swept = footprint_distance(points, (pose[0], pose[1], pose[2] + sign * amount), footprint)
        second = min(2 * amount, safe_angle)
        if second > amount + 1e-6:
            swept = min(swept, footprint_distance(
                points, (pose[0], pose[1], pose[2] + sign * second), footprint))
        options.append(dict(sign=sign, safe_angle=safe_angle,
                            opening=best_opening, clearance=clearance,
                            swept=swept, first_opening=first_opening))
    return options


def select_spin(options, previous=0, preferred=1):
    """Keep the chosen direction within one action; never undo a micro-step.

    If it becomes blocked, abort and return control to the BT for repositioning.
    Initial ranking (2026-10-20): when either side gets closer than 8 cm, the
    side that keeps the footprint farther from lethal cells wins; otherwise
    reachable exit, continuous swept clearance, reachable angle, forward
    opening, then requested direction only for a geometric tie.
    """
    if previous:
        return next((o for o in options if o['sign'] == previous), None)
    if not options:
        return None
    tight = 0.08  # abaixo de 8 cm de folga, seguranca decide antes da abertura
    return max(options, key=lambda o: (
        # 1) em espaco apertado: o sentido que mantem o contorno mais longe
        int(round(min(o.get('swept', 0.0), tight) / 0.005)),
        o['first_opening'] is not None,
        # 2) folga continua em degraus de 5 mm (antes: degraus de 1 cm ate 4 cm)
        int(round(o.get('swept', 0.0) / 0.005)),
        round(o['clearance'], 3),
        round(o['safe_angle'], 3),
        round(o['opening'], 3),
        -(o['first_opening'] if o['first_opening'] is not None else 10),
        o['sign'] == preferred))
