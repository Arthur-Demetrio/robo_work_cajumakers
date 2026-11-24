"""Smooth matched pixel segments, never retain missing detections."""
import math
import numpy as np


class SegmentSmoother:
    def __init__(self, alpha=0.45):
        self.alpha = alpha
        self.previous = []

    def update(self, segments):
        result = []
        available = list(self.previous)
        for p1, p2, _ in segments:
            current = np.asarray([p1, p2], dtype=float)
            axis = current[1] - current[0]
            length = np.linalg.norm(axis)
            best = None
            for index, old in enumerate(available):
                old_axis = old[1] - old[0]
                old_length = np.linalg.norm(old_axis)
                if min(length, old_length) < 1:
                    continue
                if abs(np.dot(axis, old_axis) / (length * old_length)) < math.cos(math.radians(12)):
                    continue
                aligned = old if np.dot(axis, old_axis) >= 0 else old[::-1]
                distance = np.linalg.norm(current - aligned, axis=1).max()
                if distance < 50 and (best is None or distance < best[0]):
                    best = distance, index, aligned
            if best is not None:
                _, index, aligned = best
                current = self.alpha * current + (1-self.alpha) * aligned
                available.pop(index)
            result.append(current)
        self.previous = result
        return [(tuple(int(v) for v in points[0]), tuple(int(v) for v in points[1]),
                 math.degrees(math.atan2(*(points[1]-points[0])[::-1]))) for points in result]


class PreviewDistanceFilter:
    """Short preview-only median; clear immediately on loss or a large step."""
    def __init__(self):
        self.values = []

    def update(self, value):
        if value is None:
            self.values.clear()
            return None
        if self.values and abs(value-self.values[-1]) > .03:
            self.values.clear()
        self.values = (self.values + [float(value)])[-3:]
        return float(np.median(self.values))
