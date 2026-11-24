import math
from typing import List, Sequence, Tuple


def conservative_clearing_upsample(
    ranges: Sequence[float],
    intensities: Sequence[float],
    factor: int,
) -> Tuple[List[float], List[float]]:
    """Subdivide only sectors proven free by both neighboring beams.

    Original beams are preserved exactly. Between two positive-infinite
    returns, synthetic positive-infinite beams may safely clear free space.
    If either neighbor is finite or unknown, synthetic beams are NaN so they
    cannot raytrace through a real obstacle.
    """
    if factor < 1:
        raise ValueError("clearing_upsample_factor precisa ser >= 1")

    count = len(ranges)
    source_intensities = (
        list(intensities) if len(intensities) == count else [0.0] * count
    )
    if count <= 1 or factor == 1:
        return list(ranges), source_intensities

    dense_ranges: List[float] = []
    dense_intensities: List[float] = []
    dense_count = (count - 1) * factor + 1
    for dense_index in range(dense_count):
        left_index, remainder = divmod(dense_index, factor)
        if remainder == 0:
            dense_ranges.append(ranges[left_index])
            dense_intensities.append(source_intensities[left_index])
            continue

        right_index = left_index + 1
        left = ranges[left_index]
        right = ranges[right_index]
        sector_is_free = (
            math.isinf(left)
            and left > 0.0
            and math.isinf(right)
            and right > 0.0
        )
        dense_ranges.append(math.inf if sector_is_free else math.nan)
        dense_intensities.append(0.0)

    return dense_ranges, dense_intensities
