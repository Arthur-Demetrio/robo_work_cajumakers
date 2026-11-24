"""ROS-independent confirmation and conservative rasterization of tape walls."""
import math
import numpy as np


class WallTracker:
    def __init__(self, confirmations=3, tolerance=.08, max_walls=2000,
                 confirmation_tolerance=.025, min_length=.08):
        self.confirmations = confirmations
        self.tolerance = tolerance
        self.max_walls = max_walls
        self.confirmation_tolerance = confirmation_tolerance
        self.min_length = min_length
        self.pending = []
        self.walls = []

    def matches(self, a, b):
        direct = np.linalg.norm(a-b, axis=1).max()
        reverse = np.linalg.norm(a-b[::-1], axis=1).max()
        return min(direct, reverse) <= self.tolerance

    @staticmethod
    def aligned(points, reference):
        if np.linalg.norm(points-reference,axis=1).max() > np.linalg.norm(points-reference[::-1],axis=1).max():
            return points[::-1]
        return points

    @staticmethod
    def same_direction(a, b):
        u,v = a[1]-a[0],b[1]-b[0]
        denominator = np.linalg.norm(u)*np.linalg.norm(v)
        return denominator > 1e-9 and abs(u@v)/denominator >= math.cos(math.radians(10))

    def covered(self, points, wall):
        if not self.same_direction(points,wall):
            return False
        axis = wall[1]-wall[0]
        length = np.linalg.norm(axis)
        axis /= length
        offsets = points-wall[0]
        across = np.abs(offsets @ np.array([-axis[1],axis[0]]))
        along = offsets @ axis
        return (across.max() <= self.confirmation_tolerance and
                along.min() >= -self.tolerance and along.max() <= length+self.tolerance)

    def update(self, segments):
        available = list(self.pending)
        pending = []
        added = False
        for points in segments:
            points = np.asarray(points, dtype=float)
            if points.shape != (2,2) or not np.isfinite(points).all():
                continue
            if np.linalg.norm(points[1]-points[0]) < self.min_length:
                continue
            if any(self.same_direction(points,old) and
                   (self.matches(points,old) or self.covered(points,old)) for old in self.walls):
                continue
            if any(self.matches(points,old) for old, _ in pending):
                continue
            history = [points.copy()]
            for i, (old, samples) in enumerate(available):
                aligned = self.aligned(points,old)
                # Compare to both the first sample and the current median:
                # pairwise association alone lets a drifting candidate walk.
                if (self.same_direction(aligned,samples[0]) and
                        np.linalg.norm(aligned-old,axis=1).max() <= self.confirmation_tolerance and
                        np.linalg.norm(aligned-samples[0],axis=1).max() <= self.confirmation_tolerance):
                    history = samples+[aligned.copy()]
                    available.pop(i)
                    break
            stable = np.median(np.stack(history),axis=0)
            if len(history) >= self.confirmations:
                if len(self.walls) < self.max_walls:
                    self.walls.append(stable)
                    added = True
            else:
                pending.append((stable,history))
        self.pending = pending
        return added


def rasterize(walls, width, height, resolution, origin, half_width):
    """Mark every cell potentially intersected by the thick tape segment.

    half_width includes half tape width plus metric uncertainty, not footprint.
    Half a cell diagonal is added conservatively for raster discretization.
    Origin is (x, y, yaw); rotated OccupancyGrid origins are supported.
    """
    if width <= 0 or height <= 0 or resolution <= 0 or half_width < 0:
        raise ValueError('Invalid mask geometry')
    grid = np.zeros((height,width),dtype=np.int8)
    x,y,yaw = origin
    rotation = np.array([[math.cos(yaw),-math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    radius = half_width/resolution + math.sqrt(.5)
    for wall in walls:
        points = (np.asarray(wall)-[x,y]) @ rotation / resolution
        lower = np.maximum(np.floor(points.min(axis=0)-radius).astype(int),0)
        upper = np.minimum(np.ceil(points.max(axis=0)+radius).astype(int),[width-1,height-1])
        if (upper < lower).any():
            continue
        xx,yy = np.meshgrid(np.arange(lower[0],upper[0]+1)+.5,
                            np.arange(lower[1],upper[1]+1)+.5)
        cells = np.stack((xx,yy),axis=-1)
        axis = points[1]-points[0]
        length2 = float(axis @ axis)
        if length2 < 1e-12:
            continue
        t = np.clip(((cells-points[0]) @ axis)/length2,0,1)
        distance = np.linalg.norm(cells-(points[0]+t[...,None]*axis),axis=-1)
        patch = grid[lower[1]:upper[1]+1,lower[0]:upper[0]+1]
        patch[distance <= radius] = 100
    return grid
