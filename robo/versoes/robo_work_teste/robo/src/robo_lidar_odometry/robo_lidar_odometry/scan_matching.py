"""Nucleo numerico independente de ROS para scan matching 2D."""

from dataclasses import dataclass
import math
import time
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MatchResult:
    pose: Pose2D
    rmse: float
    correspondences: int
    iterations: int
    converged: bool


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def compose(left: Pose2D, right: Pose2D) -> Pose2D:
    """Compoe transformacoes rigidas: resultado = left * right."""
    cosine = math.cos(left.yaw)
    sine = math.sin(left.yaw)
    return Pose2D(
        left.x + cosine * right.x - sine * right.y,
        left.y + sine * right.x + cosine * right.y,
        wrap_angle(left.yaw + right.yaw),
    )


def relative(reference: Pose2D, current: Pose2D) -> Pose2D:
    """Retorna a pose de current expressa no referencial reference."""
    dx = current.x - reference.x
    dy = current.y - reference.y
    cosine = math.cos(reference.yaw)
    sine = math.sin(reference.yaw)
    return Pose2D(
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        wrap_angle(current.yaw - reference.yaw),
    )


def transform_points(points: np.ndarray, pose: Pose2D) -> np.ndarray:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + np.array([pose.x, pose.y])


def _correspondences(
    reference: np.ndarray,
    transformed: np.ndarray,
    maximum_distance: float,
    trim_fraction: float,
    minimum_count: int,
):
    differences = transformed[:, None, :] - reference[None, :, :]
    squared = np.einsum("ijk,ijk->ij", differences, differences)
    nearest_indices = np.argmin(squared, axis=1)
    nearest_squared = squared[np.arange(len(transformed)), nearest_indices]
    valid = np.flatnonzero(nearest_squared <= maximum_distance**2)
    if len(valid) < minimum_count:
        return None
    keep = max(minimum_count, int(math.ceil(len(valid) * trim_fraction)))
    order = np.argsort(nearest_squared[valid])[:keep]
    source_indices = valid[order]
    target_indices = nearest_indices[source_indices]
    return source_indices, target_indices, nearest_squared[source_indices]


def _best_fit(source: np.ndarray, target: np.ndarray) -> Pose2D:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    centered_source = source - source_center
    centered_target = target - target_center
    covariance = centered_source.T @ centered_target
    left, _singular, right_t = np.linalg.svd(covariance)
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_t[-1, :] *= -1.0
        rotation = right_t.T @ left.T
    translation = target_center - rotation @ source_center
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return Pose2D(float(translation[0]), float(translation[1]), yaw)


def _pose_score(
    reference: np.ndarray,
    current: np.ndarray,
    pose: Pose2D,
    maximum_distance: float,
    trim_fraction: float,
    minimum_count: int,
) -> Optional[tuple[float, int]]:
    transformed = transform_points(current, pose)
    matches = _correspondences(
        reference,
        transformed,
        maximum_distance,
        trim_fraction,
        minimum_count,
    )
    if matches is None:
        return None
    _source, _target, squared = matches
    return float(math.sqrt(float(np.mean(squared)))), len(squared)


def correlative_guess(
    reference: np.ndarray,
    current: np.ndarray,
    initial: Pose2D,
    maximum_distance: float,
    trim_fraction: float,
    minimum_count: int,
    linear_window: float,
    linear_step: float,
    angular_window: float,
    angular_step: float,
) -> Pose2D:
    """Busca local coarse-to-fine ao redor da previsao antes do ICP."""

    def search(
        center: Pose2D,
        x_offsets: np.ndarray,
        y_offsets: np.ndarray,
        yaw_offsets: np.ndarray,
    ) -> Pose2D:
        best_pose = center
        best_score = math.inf

        for yaw_offset in yaw_offsets:
            for x_offset in x_offsets:
                for y_offset in y_offsets:
                    candidate = Pose2D(
                        center.x + float(x_offset),
                        center.y + float(y_offset),
                        wrap_angle(center.yaw + float(yaw_offset)),
                    )

                    scored = _pose_score(
                        reference,
                        current,
                        candidate,
                        maximum_distance,
                        trim_fraction,
                        minimum_count,
                    )

                    if scored is None:
                        continue

                    rmse, _count = scored

                    departure = math.hypot(
                        float(x_offset),
                        float(y_offset),
                    )
                    departure += 0.5 * abs(float(yaw_offset))

                    score = rmse + 0.10 * departure

                    if score < best_score:
                        best_score = score
                        best_pose = candidate

        return best_pose

    # Etapa 1: busca grossa em toda a janela original.
    coarse_linear_step = max(linear_step * 2.0, linear_step)
    coarse_angular_step = max(angular_step * 2.0, angular_step)

    coarse_x = np.arange(
        -linear_window,
        linear_window + 0.5 * coarse_linear_step,
        coarse_linear_step,
    )

    coarse_y = coarse_x

    coarse_yaw = np.arange(
        -angular_window,
        angular_window + 0.5 * coarse_angular_step,
        coarse_angular_step,
    )

    coarse_pose = search(
        initial,
        coarse_x,
        coarse_y,
        coarse_yaw,
    )

    # Etapa 2: busca fina somente ao redor do melhor candidato.
    fine_x = np.arange(
        -linear_step,
        linear_step + 0.5 * linear_step,
        linear_step,
    )

    fine_y = fine_x

    fine_yaw = np.arange(
        -angular_step,
        angular_step + 0.5 * angular_step,
        angular_step,
    )

    fine_pose = search(
        coarse_pose,
        fine_x,
        fine_y,
        fine_yaw,
    )

    return fine_pose


def match_scans(
    reference: np.ndarray,
    current: np.ndarray,
    initial: Pose2D,
    *,
    maximum_distance: float = 0.30,
    trim_fraction: float = 0.75,
    minimum_count: int = 20,
    maximum_iterations: int = 12,
    linear_window: float = 0.08,
    linear_step: float = 0.02,
    angular_window: float = 0.08,
    angular_step: float = 0.02,
) -> Optional[MatchResult]:
    """Alinha pontos do scan atual no scan anterior.

    A pose retornada e o movimento do sensor/base atual expresso no
    referencial do scan anterior.
    """
    if reference.ndim != 2 or current.ndim != 2:
        raise ValueError("scans devem ser matrizes Nx2")
    if reference.shape[1:] != (2,) or current.shape[1:] != (2,):
        raise ValueError("scans devem ser matrizes Nx2")
    if len(reference) < minimum_count or len(current) < minimum_count:
        return None
    correlative_start = time.perf_counter()
    estimate = correlative_guess(
        reference,
        current,
        initial,
        maximum_distance,
        trim_fraction,
        minimum_count,
        linear_window,
        linear_step,
        angular_window,
        angular_step,
    )
    correlative_elapsed = time.perf_counter() - correlative_start
    correlative_estimate = estimate

    icp_start = time.perf_counter()
    converged = False
    completed_iterations = 0
    for iteration in range(maximum_iterations):
        completed_iterations = iteration + 1
        transformed = transform_points(current, estimate)
        matches = _correspondences(
            reference,
            transformed,
            maximum_distance,
            trim_fraction,
            minimum_count,
        )
        if matches is None:
            return None
        source_indices, target_indices, _squared = matches
        correction = _best_fit(
            transformed[source_indices], reference[target_indices]
        )
        estimate = compose(correction, estimate)
        if (
            math.hypot(correction.x, correction.y) < 0.0005
            and abs(correction.yaw) < 0.0005
        ):
            converged = True
            break
    icp_elapsed = time.perf_counter() - icp_start
    print(
    "DEBUG MATCH | "
    f"initial=({initial.x:+.4f}, {initial.y:+.4f}, {initial.yaw:+.4f}) | "
    f"correlativo=({correlative_estimate.x:+.4f}, "
    f"{correlative_estimate.y:+.4f}, "
    f"{correlative_estimate.yaw:+.4f}) | "
    f"ICP=({estimate.x:+.4f}, {estimate.y:+.4f}, {estimate.yaw:+.4f})",
    flush=True,
)

    print(
        f"PERF MATCH | correlativo={correlative_elapsed * 1000.0:.1f} ms | "
        f"ICP={icp_elapsed * 1000.0:.1f} ms | "
        f"iteracoes={completed_iterations}",
        flush=True,
    )
    scored = _pose_score(
        reference,
        current,
        estimate,
        maximum_distance,
        trim_fraction,
        minimum_count,
    )
    if scored is None:
        return None
    rmse, count = scored
    return MatchResult(
        pose=estimate,
        rmse=rmse,
        correspondences=count,
        iterations=completed_iterations,
        converged=converged,
    )
