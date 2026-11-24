import math

import pytest

from robo_lidar.scan_utils import conservative_clearing_upsample


def test_original_finite_returns_are_preserved_without_synthetic_clearing():
    ranges, intensities = conservative_clearing_upsample(
        [1.0, 2.0], [10.0, 20.0], 4
    )

    assert ranges[0] == pytest.approx(1.0)
    assert ranges[-1] == pytest.approx(2.0)
    assert all(math.isnan(value) for value in ranges[1:-1])
    assert intensities == [10.0, 0.0, 0.0, 0.0, 20.0]


def test_only_two_free_neighbors_create_free_subrays():
    free_ranges, _ = conservative_clearing_upsample(
        [math.inf, math.inf], [], 4
    )
    mixed_ranges, _ = conservative_clearing_upsample(
        [math.inf, 1.0], [], 4
    )

    assert all(math.isinf(value) and value > 0.0 for value in free_ranges)
    assert math.isinf(mixed_ranges[0])
    assert all(math.isnan(value) for value in mixed_ranges[1:-1])
    assert mixed_ranges[-1] == pytest.approx(1.0)


def test_factor_one_keeps_scan_unchanged():
    ranges, intensities = conservative_clearing_upsample(
        [math.nan, 1.5, math.inf], [0.0, 2.0, 0.0], 1
    )

    assert math.isnan(ranges[0])
    assert ranges[1] == pytest.approx(1.5)
    assert math.isinf(ranges[2])
    assert intensities == [0.0, 2.0, 0.0]


def test_invalid_factor_is_rejected():
    with pytest.raises(ValueError):
        conservative_clearing_upsample([math.inf], [], 0)
