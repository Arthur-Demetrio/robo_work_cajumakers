"""Detector de linhas com limiar angular adaptativo, sem bibliotecas externas.

O limiar adapta APENAS o agrupamento inicial; ajuste de reta continua
exigindo residual baixo, evitando tratar pontos agrupados como prova de caixa.
"""
import math


def adaptive_gap(first, second, angle_increment, min_gap=0.13,
                 noise_allowance=0.035, angular_factor=1.8, max_gap=0.34):
    """Raio angular: distancias de feixes consecutivos crescem com alcance."""
    near = min(math.hypot(*first), math.hypot(*second))
    separation = 2.0 * near * math.sin(abs(angle_increment)/2.0)
    return max(min_gap, min(max_gap, angular_factor*separation+noise_allowance))


def extract_segments(points, angle_increment, max_gap=0.13, min_points=6,
                     max_residual=0.035, adaptive=True):
    clusters=[];current=[]
    for point in points:
        if point is None or not all(map(math.isfinite,point)):
            if current:clusters.append(current);current=[]
            continue
        if current:
            limit=(adaptive_gap(current[-1],point,angle_increment,min_gap=max_gap)
                   if adaptive else max_gap)
            if math.dist(point,current[-1])>limit:
                clusters.append(current);current=[]
        current.append(point)
    if current:clusters.append(current)
    segments=[]
    for cluster in clusters:
        if len(cluster)>=min_points:
            _split(cluster,min_points,max_residual,segments)
    return segments


def _split(points,min_points,residual_limit,out):
    if len(points)<min_points:return
    a,b=points[0],points[-1]
    length=math.dist(a,b)
    if length<1e-5:return
    deviations=[abs((b[0]-a[0])*(a[1]-p[1])-(a[0]-p[0])*(b[1]-a[1]))/length for p in points]
    index=max(range(len(points)),key=deviations.__getitem__)
    if deviations[index]>residual_limit:
        if index>=min_points-1 and len(points)-index>=min_points:
            _split(points[:index+1],min_points,residual_limit,out)
            _split(points[index:],min_points,residual_limit,out)
        # Nao forcar segmentos de 4 pontos: aumenta falsos positivos em parede.
        return
    out.append((a,b,len(points),max(deviations)))
