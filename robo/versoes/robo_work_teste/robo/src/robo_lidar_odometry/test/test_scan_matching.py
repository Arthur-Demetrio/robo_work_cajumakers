import math

import numpy as np

from robo_lidar_odometry.scan_matching import (
    Pose2D,
    compose,
    match_scans,
    transform_points,
)


def _environment():
    horizontal = np.column_stack((np.linspace(-2.5, 2.5, 101), np.full(101, 3.0)))
    vertical = np.column_stack((np.full(81, 2.0), np.linspace(-1.0, 3.0, 81)))
    diagonal = np.column_stack((np.linspace(-1.0, 0.5, 35), np.linspace(1.0, 2.2, 35)))
    return np.vstack((horizontal, vertical, diagonal))


def _inverse(pose):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Pose2D(
        -cosine * pose.x - sine * pose.y,
        sine * pose.x - cosine * pose.y,
        -pose.yaw,
    )


def test_recovers_motion_between_scans():
    reference = _environment()
    expected = Pose2D(0.055, -0.018, 0.025)
    current = transform_points(reference, _inverse(expected))
    result = match_scans(reference, current, Pose2D(0.05, 0.0, 0.02))
    assert result is not None
    assert result.correspondences >= 100
    assert result.rmse < 0.01
    assert math.isclose(result.pose.x, expected.x, abs_tol=0.006)
    assert math.isclose(result.pose.y, expected.y, abs_tol=0.006)
    assert math.isclose(result.pose.yaw, expected.yaw, abs_tol=0.006)


def test_composition_uses_body_delta():
    pose = compose(Pose2D(1.0, 2.0, math.pi / 2), Pose2D(0.5, 0.0, 0.0))
    assert math.isclose(pose.x, 1.0, abs_tol=1e-9)
    assert math.isclose(pose.y, 2.5, abs_tol=1e-9)


def test_tolerates_noise_and_outliers_at_real_scan_size():
    angles = np.linspace(-math.pi / 2, math.pi / 2, 84)
    ranges = 2.0 + 0.6 * np.sin(3.0 * angles) + 0.3 * np.cos(7.0 * angles)
    reference = np.column_stack(
        (ranges * np.cos(angles), ranges * np.sin(angles))
    )
    expected = Pose2D(0.025, 0.006, 0.020)
    current = transform_points(reference, _inverse(expected))
    generator = np.random.default_rng(8)
    current += generator.normal(0.0, 0.004, current.shape)
    current = np.vstack(
        (current, generator.uniform([-1.5, -1.5], [1.5, 2.5], (8, 2)))
    )
    result = match_scans(
        reference,
        current,
        Pose2D(0.020, 0.0, 0.015),
        linear_window=0.06,
        angular_window=0.06,
    )
    assert result is not None
    assert result.rmse < 0.012
    assert math.isclose(result.pose.x, expected.x, abs_tol=0.008)
    assert math.isclose(result.pose.y, expected.y, abs_tol=0.008)
    assert math.isclose(result.pose.yaw, expected.yaw, abs_tol=0.008)
