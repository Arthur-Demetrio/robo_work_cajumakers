"""Medicao do corredor pelo LiDAR (sem ROS)."""
import math


def _reta(pts, tol=0.025):
    """y = a + b*x por minimos quadrados, com rejeicao de pontos fora da reta."""
    for _ in range(3):
        n = len(pts)
        if n < 6:
            return None
        mx = sum(p[0] for p in pts) / n
        my = sum(p[1] for p in pts) / n
        sxx = sum((p[0] - mx) ** 2 for p in pts)
        if sxx < 1e-4:
            return None
        b = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
        a = my - b * mx
        bons = [p for p in pts if abs(p[1] - (a + b * p[0])) < tol]
        if len(bons) == n:
            break
        pts = bons
    if len(pts) < 6:
        return None
    xs = [p[0] for p in pts]
    if max(xs) - min(xs) < 0.15 or abs(b) > 0.35:  # parede curta demais ou torta demais (>19 graus)
        return None
    return a, b, len(pts)


def medir_corredor(pts, x_min=0.20, x_max=1.40, y_min=0.22, y_max=0.80, x_ref=0.08,
                   largura_conhecida=None):
    """pts: pontos do LiDAR em base_link. Devolve dict ou None.

    angulo: quanto o robo precisa girar (rad, + = anti-horario) para ficar paralelo as paredes.
    desvio: quanto o centro do corredor esta a esquerda do centro do robo (m) em x_ref.
    """
    esq = _reta([p for p in pts if x_min <= p[0] <= x_max and y_min < p[1] < y_max])
    dir_ = _reta([p for p in pts if x_min <= p[0] <= x_max and -y_max < p[1] < -y_min])
    if esq is None and dir_ is None:
        return None
    r = dict(esq=None, dir=None, largura=None, desvio=None)
    if esq:
        r['esq'] = esq[0] + esq[1] * x_ref
    if dir_:
        r['dir'] = dir_[0] + dir_[1] * x_ref
    if esq and dir_:
        # paredes quase paralelas? (senao uma delas nao e parede do corredor)
        if abs(math.atan(esq[1]) - math.atan(dir_[1])) > math.radians(8):
            # fica com a que tem mais pontos
            if esq[2] >= dir_[2]:
                dir_, r['dir'] = None, None
            else:
                esq, r['esq'] = None, None
    if esq and dir_:
        r['angulo'] = math.atan((esq[1] * esq[2] + dir_[1] * dir_[2]) / (esq[2] + dir_[2]))
        r['largura'] = r['esq'] - r['dir']
        r['desvio'] = (r['esq'] + r['dir']) / 2.0
        r['n'] = esq[2] + dir_[2]
        r['lados'] = 2
    else:
        lado = esq or dir_
        r['angulo'] = math.atan(lado[1])
        r['n'] = lado[2]
        r['lados'] = 1
        if largura_conhecida:
            if esq:
                r['desvio'] = r['esq'] - largura_conhecida / 2.0
            else:
                r['desvio'] = r['dir'] + largura_conhecida / 2.0
    return r
