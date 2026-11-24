"""Geometria 2D pura: cadastramento e aproximação de uma bancada.

A bancada é representada pela sua BORDA FRONTAL, marcada por dois pontos,
mais um ponto do lado livre para escolher o lado correto de aproximação.
Nenhuma função neste arquivo depende de ROS.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Station:
    name: str
    p1: tuple
    p2: tuple
    free_side: tuple


@dataclass(frozen=True)
class Approach:
    center_x: float
    center_y: float
    base_x: float
    base_y: float
    yaw: float
    width: float
    normal_x: float
    normal_y: float
    front_gap_m: float


def compute_approach(station, robot_front_m=0.365, front_gap_m=0.20):
    """Retorna a pose do base_link diante da bancada, com +X apontando para ela.

    A terceira marca é um ponto no ESPAÇO LIVRE em frente à face de atendimento,
    de modo que a normal escolhida não depende da pose atual do robô.
    """
    vals = (*station.p1, *station.p2, *station.free_side, robot_front_m, front_gap_m)
    if not all(math.isfinite(float(v)) for v in vals):
        raise ValueError('coordenadas e margens devem ser finitas')
    if not 0.20 <= robot_front_m <= 2.0:
        raise ValueError('robot_front_m fora do intervalo esperado')
    if not 0.08 <= front_gap_m <= 1.50:
        raise ValueError('front_gap_m fora do intervalo permitido para pre-visualizacao')

    x1, y1 = station.p1
    x2, y2 = station.p2
    sx, sy = station.free_side
    dx, dy = x2 - x1, y2 - y1
    width = math.hypot(dx, dy)
    if width < 0.25:
        raise ValueError('borda frontal pequena demais (menos de 25 cm)')

    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    nx, ny = -dy / width, dx / width
    signed_distance = (sx - center_x) * nx + (sy - center_y) * ny
    if abs(signed_distance) < 0.10:
        raise ValueError('terceiro ponto deve ficar a pelo menos 10 cm do eixo da bancada')
    if signed_distance < 0:
        nx, ny = -nx, -ny

    distance = robot_front_m + front_gap_m
    x = center_x + nx * distance
    y = center_y + ny * distance
    yaw = math.atan2(-ny, -nx)
    return Approach(center_x, center_y, x, y, yaw, width, nx, ny, front_gap_m)


def path_length_xy(poses):
    total = 0.0
    for prev, cur in zip(poses, poses[1:]):
        ax, ay = prev
        bx, by = cur
        total += math.hypot(bx - ax, by - ay)
    return total
