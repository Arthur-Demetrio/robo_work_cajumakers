"""Confere se a pegada (footprint) do robo cabe livre no costmap, sem ROS.

Usado ao planejar/finalizar: um local cuja pose deixa o robo dentro de
parede, de area inflada ou de area desconhecida faria o Nav2 falhar no meio
da missao. Mesma regra conservadora do robo_station_manager
(costmap_guard.footprint_is_clear): qualquer celula desconhecida ou com
custo alto dentro da pegada (mais uma margem) reprova.

NAO substitui o Collision Monitor: e so uma conferencia antes de sair.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from robo_locais.locais import Pose2D


@dataclass(frozen=True)
class Pegada:
    # Retangulo do base_link (docs/geometria.md do Robo-WORK).
    frente: float = 0.365
    tras: float = -0.205
    lado: float = 0.315


@dataclass(frozen=True)
class Grade:
    """Os campos do nav_msgs/OccupancyGrid que importam aqui."""

    dados: Sequence[int]
    largura: int
    altura: int
    resolucao: float
    origem_x: float
    origem_y: float
    origem_yaw: float = 0.0


def pegada_livre(
    grade: Grade,
    pose: Pose2D,
    pegada: Pegada = Pegada(),
    margem: float = 0.05,
    limiar_ocupado: int = 80,
) -> Tuple[bool, str]:
    """(livre, motivo). Varre as celulas cujo centro cai na pegada + margem."""
    if grade.resolucao <= 0 or len(grade.dados) != grade.largura * grade.altura:
        return False, "costmap invalido"
    # Meia diagonal da celula: nao deixa buraco junto a borda da pegada.
    folga = margem + grade.resolucao / math.sqrt(2)
    x_min, x_max = pegada.tras - folga, pegada.frente + folga
    y_max = pegada.lado + folga

    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    oc, os_ = math.cos(grade.origem_yaw), math.sin(grade.origem_yaw)

    def para_celula(wx: float, wy: float) -> Tuple[float, float]:
        dx, dy = wx - grade.origem_x, wy - grade.origem_y
        return (oc * dx + os_ * dy) / grade.resolucao, (-os_ * dx + oc * dy) / grade.resolucao

    cantos = [
        para_celula(pose.x + c * bx - s * by, pose.y + s * bx + c * by)
        for bx in (x_min, x_max) for by in (-y_max, y_max)
    ]
    i0 = math.floor(min(i for i, _ in cantos)) - 1
    i1 = math.floor(max(i for i, _ in cantos)) + 1
    j0 = math.floor(min(j for _, j in cantos)) - 1
    j1 = math.floor(max(j for _, j in cantos)) + 1
    if i0 < 0 or j0 < 0 or i1 >= grade.largura or j1 >= grade.altura:
        return False, "pegada fora da area conhecida do costmap"

    for j in range(j0, j1 + 1):
        for i in range(i0, i1 + 1):
            gx, gy = (i + 0.5) * grade.resolucao, (j + 0.5) * grade.resolucao
            dx = grade.origem_x + oc * gx - os_ * gy - pose.x
            dy = grade.origem_y + os_ * gx + oc * gy - pose.y
            bx, by = c * dx + s * dy, -s * dx + c * dy
            if not (x_min <= bx <= x_max and -y_max <= by <= y_max):
                continue
            custo = grade.dados[j * grade.largura + i]
            if custo < 0:
                return False, "area desconhecida sob o robo"
            if custo >= limiar_ocupado:
                return False, f"obstaculo/area inflada sob o robo (custo {custo})"
    return True, "pegada livre"
