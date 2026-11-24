"""Reimplementacao em Python (para teste offline) das checagens de caminho do Nav2 Jazzy 1.3.x e da
checagem proposta no Patch 01. Sem ROS: entra grade de custo (numpy uint8), metadados e caminho.

Semantica Nav2 reproduzida:
  - FootprintCollisionChecker::footprintCost: vertices do footprint transformado; vertice fora do mapa ->
    LETHAL; custo = maximo das celulas nas ARESTAS (LineIterator / Bresenham).
  - SmoothPath check_for_collisions: invalido se footprintCost >= 254 (LETHAL) em QUALQUER pose (inclui a
    pose do robo), orientacao das poses aproximada pelo smoother.
  - IsPathValid (planner_server Jazzy): a partir da pose mais proxima do robo; footprint com o YAW GRAVADO na
    pose (NavFn grava 0 em todas menos a ultima); invalido se custo == 254 ou 253.
"""
import math
import numpy as np

LETHAL, INSCRITO, DESCONHECIDO = 254, 253, 255
FOOTPRINT = [(0.375, 0.325), (0.375, -0.325), (-0.215, -0.325), (-0.215, 0.325)]  # com padding 0.01


def mundo_p_mapa(meta, x, y):
    sx, sy, res, ox, oy = meta
    if x < ox or y < oy:
        return None
    mx, my = int((x - ox) / res), int((y - oy) / res)
    if mx >= sx or my >= sy:
        return None
    return mx, my


def linha(x0, y0, x1, y1):
    """Bresenham como o nav2_util::LineIterator."""
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 >= x0 else -1), (1 if y1 >= y0 else -1)
    x, y = x0, y0
    if dx >= dy:
        err = dx // 2
        for _ in range(dx + 1):
            yield x, y
            err -= dy
            if err < 0:
                y += sy; err += dx
            x += sx
    else:
        err = dy // 2
        for _ in range(dy + 1):
            yield x, y
            err -= dx
            if err < 0:
                x += sx; err += dy
            y += sy


def custo_footprint(grade, meta, x, y, th, fp=FOOTPRINT, ignorar_desconhecido=False):
    """Como FootprintCollisionChecker::footprintCost: maximo nas arestas, retorno imediato no LETHAL.
    Celula 255 entra no maximo (o Nav2 nao a trata a parte); com ignorar_desconhecido ela conta 0."""
    c, s = math.cos(th), math.sin(th)
    vs = []
    for (a, b) in fp:
        m = mundo_p_mapa(meta, x + c*a - s*b, y + s*a + c*b)
        if m is None:
            return LETHAL
        vs.append(m)
    mx = 0
    for i in range(len(vs)):
        (x0, y0), (x1, y1) = vs[i], vs[(i + 1) % len(vs)]
        for (cx, cy) in linha(x0, y0, x1, y1):
            v = int(grade[cy, cx])
            if v == LETHAL:
                return LETHAL
            if v == DESCONHECIDO and ignorar_desconhecido:
                v = 0
            if v > mx:
                mx = v
    return mx


def custo_centro(grade, meta, x, y):
    m = mundo_p_mapa(meta, x, y)
    return LETHAL if m is None else int(grade[m[1], m[0]])


def yaw_tangente(pts, yaw_final=None):
    """Orientacao aproximada pela tangente (como updateApproximatePathOrientations)."""
    n = len(pts)
    out = []
    for i in range(n):
        j0, j1 = (i, i + 1) if i < n - 1 else (i - 1, i)
        if n < 2:
            out.append(pts[i][2]); continue
        out.append(math.atan2(pts[j1][1] - pts[j0][1], pts[j1][0] - pts[j0][0]))
    if yaw_final is not None and n:
        out[-1] = yaw_final
    return out


def indice_mais_proximo(pts, rx, ry):
    best, bi = 1e18, 0
    for i, p in enumerate(pts):
        d = (p[0] - rx)**2 + (p[1] - ry)**2
        if d < best:
            best, bi = d, i
    return bi


def is_path_valid_jazzy(grade, meta, pts, rx, ry):
    """Semantica do planner_server Jazzy (footprint, yaw gravado). Retorna (valido, indice)."""
    for i in range(indice_mais_proximo(pts, rx, ry), len(pts)):
        c = custo_footprint(grade, meta, pts[i][0], pts[i][1], pts[i][2])
        if c in (LETHAL, INSCRITO):
            return False, i
    return True, None


def colisao_estilo_smoother(grade, meta, pts, yaws):
    """footprint >= LETHAL em qualquer pose (o smoother Jazzy considera 255 >= 254 como colisao)."""
    for i, p in enumerate(pts):
        if custo_footprint(grade, meta, p[0], p[1], yaws[i]) >= LETHAL:
            return False, i
    return True, None


def validar_original(grade, meta, pts, rx, ry, ryaw, yaw_goal, folga_inicio=0.30, folga_fim=0.0,
                     desconhecido_livre=True):
    """CHECAGEM PROPOSTA (Patch 01) para o caminho ORIGINAL do planner quando a suavizacao falhou.

    - so o trecho restante (a partir da pose mais proxima do robo);
    - ignora os primeiros `folga_inicio` m: e onde o robo JA esta (o footprint atual pode encostar no
      custo letal da inflacao/ruido do LiDAR; o Collision Monitor e o destravar cuidam disso);
    - orientacao pela tangente (NavFn grava yaw 0), ultima pose com o yaw do goal;
    - invalido so com custo LETHAL (obstaculo real sob o footprint), nao INSCRITO (inflacao): e o mesmo
      criterio do smoother, que e o que o robo fisicamente nao pode atravessar;
    - celula desconhecida nao invalida (allow_unknown: true no planner).
    Retorna (valido, indice_da_colisao, motivo)."""
    if not pts:
        return False, None, 'caminho vazio'
    i0 = indice_mais_proximo(pts, rx, ry)
    yaws = yaw_tangente(pts, yaw_goal)
    s = 0.0
    for i in range(i0, len(pts)):
        if i > i0:
            s += math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
        if s < folga_inicio:
            continue
        c = custo_footprint(grade, meta, pts[i][0], pts[i][1], yaws[i], ignorar_desconhecido=desconhecido_livre)
        if c >= LETHAL:
            return False, i, 'footprint em obstaculo a %.2f m do robo' % s
    return True, None, 'ok'
