"""Verificacao local conservadora de footprint no OccupancyGrid do costmap.

Independente de ROS para permitir testes unitarios. NAO e um substituto do
Collision Monitor nem garante ausencia de obstaculos fora do plano do LiDAR.
"""
import math


def footprint_is_clear(values, width, height, resolution, origin_x, origin_y,
                       origin_yaw, robot_x, robot_y, robot_yaw,
                       front=0.365, rear=-0.205, side=0.315,
                       extra_margin=0.05, occupied_threshold=80):
    """Retorna (livre, motivo). Qualquer amostra desconhecida e rejeitada.

    Usa centro de celula com expansao pela meia diagonal, para nao deixar
    lacunas junto a borda do footprint. Esta validacao assume retangulo mecanum.
    """
    nums = (resolution, origin_x, origin_y, origin_yaw, robot_x, robot_y,
            robot_yaw, front, rear, side, extra_margin)
    if not all(math.isfinite(v) for v in nums):
        return False, 'valores nao finitos no costmap/footprint'
    if (width < 1 or height < 1 or resolution <= 0 or len(values) != width * height
            or front <= 0 or rear >= 0 or side <= 0 or extra_margin < 0):
        return False, 'geometria ou dados do costmap invalidos'

    # Amostra celulas cujos centros se encontram dentro do footprint expandido
    # com margem adicional de meia diagonal de celula.
    pad = extra_margin + resolution / math.sqrt(2)
    x_min, x_max = rear - pad, front + pad
    y_max = side + pad
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    corners = []
    for bx in (x_min, x_max):
        for by in (-y_max, y_max):
            wx = robot_x + c * bx - s * by
            wy = robot_y + s * bx + c * by
            corners.append((wx, wy))

    oc, os = math.cos(origin_yaw), math.sin(origin_yaw)

    def grid_continuous(wx, wy):
        dx, dy = wx - origin_x, wy - origin_y
        return ((oc * dx + os * dy) / resolution,
                (-os * dx + oc * dy) / resolution)

    g = [grid_continuous(wx, wy) for wx, wy in corners]
    imin = math.floor(min(x for x, _ in g)) - 1
    imax = math.floor(max(x for x, _ in g)) + 1
    jmin = math.floor(min(y for _, y in g)) - 1
    jmax = math.floor(max(y for _, y in g)) + 1
    if imin < 0 or jmin < 0 or imax >= width or jmax >= height:
        return False, 'footprint ou margem fora do costmap conhecido'

    for j in range(jmin, jmax + 1):
        for i in range(imin, imax + 1):
            gx, gy = (i + 0.5) * resolution, (j + 0.5) * resolution
            wx = origin_x + oc * gx - os * gy
            wy = origin_y + os * gx + oc * gy
            dx, dy = wx - robot_x, wy - robot_y
            bx, by = c * dx + s * dy, -s * dx + c * dy
            if x_min <= bx <= x_max and -y_max <= by <= y_max:
                cost = values[j * width + i]
                if cost < 0:
                    return False, f'area desconhecida no footprint (celula {i},{j})'
                if cost >= occupied_threshold:
                    return False, f'custo elevado no footprint (celula {i},{j}, custo {cost})'
    return True, 'footprint livre no costmap global amostrado'
