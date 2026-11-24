"""Pure-Python geometric inference and preview-only velocity guard.

No ROS dependency; physical movement is NEVER requested by this module.
All lengths are metres, velocities m/s and rad/s.
"""
from dataclasses import dataclass, replace
import math
from statistics import median
from typing import List, Optional, Tuple

Point = Tuple[float, float]

@dataclass(frozen=True)
class Wall:
    slope: float
    offset: float
    inliers: int
    residual_m: float
    xspan_m: float

@dataclass(frozen=True)
class Corridor:
    detected: bool
    width_m: Optional[float] = None
    center_y_m: Optional[float] = None
    heading_rad: Optional[float] = None
    left: Optional[Wall] = None
    right: Optional[Wall] = None
    reason: str = 'SEM_LEITURA'
    physical_clearance_m: Optional[float] = None

@dataclass(frozen=True)
class Config:
    robot_width_m: float = 0.630
    min_width_m: float = 0.70
    max_width_m: float = 1.05
    side_min_y_m: float = 0.20
    side_max_y_m: float = 0.80
    x_min_m: float = -0.25
    x_max_m: float = 0.75
    min_side_samples: int = 6
    min_x_span_m: float = 0.25
    max_residual_m: float = 0.045
    max_parallel_error_rad: float = 0.13
    min_clearance_m: float = 0.045
    large_heading_error_rad: float = 0.12
    max_turn_in_corridor_rad_s: float = 0.065
    reverse_turn_epsilon_rad_s: float = 0.015


def _linear_fit(points: List[Point]):
    n=len(points)
    avgx=sum(p[0] for p in points)/n
    avgy=sum(p[1] for p in points)/n
    den=sum((x-avgx)**2 for x,_ in points)
    if den <= 1.e-7:
        return None
    slope=sum((x-avgx)*(y-avgy) for x,y in points)/den
    return slope, avgy-slope*avgx


def _fit_wall(points: List[Point], cfg: Config) -> Optional[Wall]:
    if len(points) < cfg.min_side_samples:
        return None
    # Two passes of geometric residual trimming. This resists isolated scan returns.
    kept=list(points)
    for _ in range(2):
        coeff=_linear_fit(kept)
        if coeff is None:
            return None
        a,b=coeff
        errors=[abs(y-(a*x+b))/math.sqrt(1+a*a) for x,y in kept]
        robust_scale=median(errors)
        cutoff=max(0.018, min(cfg.max_residual_m, 3.0*robust_scale+0.009))
        filtered=[p for p,e in zip(kept,errors) if e<=cutoff]
        if len(filtered)<cfg.min_side_samples:
            return None
        kept=filtered
    coeff=_linear_fit(kept)
    if coeff is None:
        return None
    a,b=coeff
    span=max(x for x,_ in kept)-min(x for x,_ in kept)
    residual=max(abs(y-(a*x+b))/math.sqrt(1+a*a) for x,y in kept)
    if span < cfg.min_x_span_m or residual > cfg.max_residual_m:
        return None
    return Wall(a,b,len(kept),residual,span)



def diagnose_wall_fit(points: List[Point], cfg: Config) -> dict:
    """Observability-only explanation for _fit_wall's exact rejection gates.

    This MUST NOT be used to approve a corridor or actuate the robot. The
    actual detector continues to call _fit_wall without any modifications.
    """
    d={'pontos_iniciais':len(points),
       'extensao_x_inicial_m':round(max((p[0] for p in points),default=0)-min((p[0] for p in points),default=0),3),
       'amplitude_y_inicial_m':round(max((p[1] for p in points),default=0)-min((p[1] for p in points),default=0),3),
       'minimo_pontos':cfg.min_side_samples,
       'minimo_extensao_x_m':cfg.min_x_span_m,
       'limite_residuo_m':cfg.max_residual_m}
    if len(points)<cfg.min_side_samples:
        d['motivo']='PONTOS_INSUFICIENTES'; return d
    kept=list(points)
    for rodada in range(1,3):
        coeff=_linear_fit(kept)
        if coeff is None:
            d['motivo']='SEM_VARIANCIA_X_RODADA_'+str(rodada); return d
        a,b=coeff
        errors=[abs(y-(a*x+b))/math.sqrt(1+a*a) for x,y in kept]
        robust_scale=median(errors)
        cutoff=max(0.018,min(cfg.max_residual_m,3.0*robust_scale+0.009))
        filtered=[p for p,e in zip(kept,errors) if e<=cutoff]
        d['rodada_'+str(rodada)]={
           'pontos_antes':len(kept),
           'pontos_apos':len(filtered),
           'inclinacao_graus':round(math.degrees(math.atan(a)),2),
           'residuo_mediano_m':round(robust_scale,4),
           'residuo_max_m':round(max(errors),4),
           'limiar_corte_m':round(cutoff,4)}
        if len(filtered)<cfg.min_side_samples:
            d['motivo']='POUCOS_INLIERS_APOS_CORTE_'+str(rodada);return d
        kept=filtered
    coeff=_linear_fit(kept)
    if coeff is None:
        d['motivo']='SEM_VARIANCIA_X_FINAL';return d
    a,b=coeff
    span=max(x for x,_ in kept)-min(x for x,_ in kept)
    residual=max(abs(y-(a*x+b))/math.sqrt(1+a*a) for x,y in kept)
    d['pontos_finais']=len(kept)
    d['extensao_x_final_m']=round(span,3)
    d['residuo_max_final_m']=round(residual,4)
    d['inclinacao_final_graus']=round(math.degrees(math.atan(a)),2)
    if span<cfg.min_x_span_m:
        d['motivo']='EXTENSAO_X_FINAL_INSUFICIENTE'
    elif residual>cfg.max_residual_m:
        d['motivo']='RESIDUO_FINAL_EXCESSIVO'
    else:
        d['motivo']='AJUSTE_APROVADO'
    return d

def find_corridor(points: List[Point], cfg: Config = Config()) -> Corridor:
    """Conservative candidate, not proof the corridor is safe to traverse.

    Points must be already expressed in base_link; makes no use of map pose.
    Does not infer wall geometry when one side is missing.
    """
    usable=[(x,y) for x,y in points if math.isfinite(x) and math.isfinite(y)
            and cfg.x_min_m <= x <= cfg.x_max_m]
    left_pts=[(x,y) for x,y in usable if cfg.side_min_y_m<=y<=cfg.side_max_y_m]
    right_pts=[(x,y) for x,y in usable if -cfg.side_max_y_m<=y<=-cfg.side_min_y_m]
    left=_fit_wall(left_pts,cfg)
    right=_fit_wall(right_pts,cfg)
    if left is None or right is None:
        return Corridor(False,reason='DUAS_PAREDES_NAO_OBSERVADAS',left=left,right=right)
    heading1=math.atan(left.slope)
    heading2=math.atan(right.slope)
    if abs(heading1-heading2)>cfg.max_parallel_error_rad:
        return Corridor(False,reason='PAREDES_NAO_PARALELAS',left=left,right=right)
    width=(left.offset-right.offset)/math.sqrt(1+((left.slope+right.slope)/2)**2)
    center=(left.offset+right.offset)/2
    heading=(heading1+heading2)/2
    clearance=(width-cfg.robot_width_m)/2
    if not (cfg.min_width_m<=width<=cfg.max_width_m):
        return Corridor(False,width,center,heading,left,right,'LARGURA_INCOMPATIVEL',clearance)
    # Note: if corridor center is offset relative to base_link, nominal clearance
    # of the actual robot is reduced by abs(center_y_m).
    side_clearance=clearance-abs(center)
    if side_clearance < cfg.min_clearance_m:
        return Corridor(True,width,center,heading,left,right,'FOLGA_INSUFICIENTE',side_clearance)
    return Corridor(True,width,center,heading,left,right,'CORREDOR_POSSIVEL',side_clearance)


def evaluate_zones(points: List[Point], cfg: Config = Config()):
    """Separate currently observed walls from forward evidence.

    The lookahead region never establishes local robot-to-wall clearance.
    Return (local, ahead); both are hypotheses, never an obstacle clearance check.
    """
    local_cfg=replace(cfg,x_min_m=-0.25,x_max_m=0.75)
    ahead_cfg=replace(cfg,x_min_m=0.75,x_max_m=1.40)
    return find_corridor(points,local_cfg), find_corridor(points,ahead_cfg)


def preview_cmd(vx:float,vy:float,wz:float, corridor:Corridor,
                scan_fresh:bool, cfg:Config=Config()):
    """What a future stop-only guard would do. PREVIEW, never drive motors."""
    if not scan_fresh:
        return (0.0,0.0,0.0),'SCAN_AUSENTE_OU_ANTIGO'
    if not corridor.detected:
        return (vx,vy,wz),'FORA_DO_MODO_CORREDOR'
    if corridor.reason=='FOLGA_INSUFICIENTE':
        return (0.0,0.0,0.0),'FOLGA_INSUFICIENTE'
    if abs(corridor.heading_rad or 0)>cfg.large_heading_error_rad:
        return (0.0,0.0,0.0),'DESALINHAMENTO_GRANDE'
    if vx < -0.01 and abs(wz)>cfg.reverse_turn_epsilon_rad_s:
        return (0.0,0.0,0.0),'RE_COM_ROTACAO'
    if abs(vy)>0.015:
        return (0.0,0.0,0.0),'DESLOCAMENTO_LATERAL_NO_CORREDOR'
    if abs(wz)>cfg.max_turn_in_corridor_rad_s:
        return (0.0,0.0,0.0),'GIRO_EXCESSIVO'
    return (vx,vy,wz),'PASSAGEM_CANDIDATA_SEM_BLOQUEIO_DE_COMANDO'


# v0.1.6: DIAGNOSTIC-ONLY segmented-line hypotheses.
# This code is NOT used to approve a corridor or control the robot.

def _line_models(points: List[Point], cfg: Config) -> list:
    """Find short x-directed line segments even when the entire side is mixed.

    Deterministic bounded pair consensus, followed by re-fitting and an x-gap
    continuity check. Candidate geometry is advisory only.
    """
    pts=sorted([(float(x),float(y)) for x,y in points
                if math.isfinite(x) and math.isfinite(y)])
    if len(pts)<cfg.min_side_samples:
        return []
    max_slope=math.tan(math.radians(13.0))
    models=[]
    for i in range(len(pts)):
        for j in range(i+1,len(pts)):
            x0,y0=pts[i]; x1,y1=pts[j]
            if x1-x0<0.23: continue
            slope=(y1-y0)/(x1-x0)
            if abs(slope)>max_slope: continue
            offset=y0-slope*x0
            dist=[(p, abs(p[1]-(slope*p[0]+offset))/math.sqrt(1+slope*slope))
                  for p in pts]
            inliers=[p for p,d in dist if d<=0.035]
            if len(inliers)<cfg.min_side_samples: continue
            # Different obstacles across a doorway must not be treated as
            # one supported line just because they are approximately collinear.
            groups=[];group=[]
            for p in inliers:
                if group and p[0]-group[-1][0]>0.19:
                    groups.append(group);group=[]
                group.append(p)
            if group: groups.append(group)
            for group in groups:
                if len(group)<cfg.min_side_samples:continue
                if group[-1][0]-group[0][0]<cfg.min_x_span_m:continue
                fit=_linear_fit(group)
                if fit is None: continue
                a,b=fit
                if abs(a)>max_slope:continue
                errors=[abs(y-(a*x+b))/math.sqrt(1+a*a) for x,y in group]
                if max(errors)>cfg.max_residual_m:continue
                xlo=min(x for x,_ in group);xhi=max(x for x,_ in group)
                score=(xhi-xlo)*(0.4+len(group)/len(pts))/(0.02+max(errors))
                models.append({'inliers':len(group),'x_min':xlo,'x_max':xhi,
                               'slope':a,'offset':b,'residual':max(errors),
                               'score':score})
    # Suppress nearly identical hypotheses, while preserving separate planes.
    models.sort(key=lambda m:m['score'],reverse=True)
    distinct=[]
    for m in models:
        if all(abs(m['offset']-q['offset'])>0.045 or
               abs(m['slope']-q['slope'])>0.055 or
               abs(m['x_min']-q['x_min'])>0.18
               for q in distinct):
            distinct.append(m)
        if len(distinct)>=5:break
    return distinct

def segmentacao_experimental(points: List[Point],cfg: Config,
                              x_min: float = 0.75, x_max: float = 1.40)->dict:
    """Advisory observation: all results are NON-AUTHORITATIVE.

    A geometric pair does not certify free space next to the body.
    """
    usable=[(x,y) for x,y in points if math.isfinite(x) and math.isfinite(y)
            and x_min<=x<=x_max]
    lp=[(x,y) for x,y in usable if cfg.side_min_y_m<=y<=cfg.side_max_y_m]
    rp=[(x,y) for x,y in usable if -cfg.side_max_y_m<=y<=-cfg.side_min_y_m]
    left=_line_models(lp,cfg);right=_line_models(rp,cfg)
    out={'modo':'DIAGNOSTICO_APENAS',
         'segmentos_esquerda':[{k:round(v,4) if isinstance(v,float) else v for k,v in m.items() if k!='score'} for m in left[:3]],
         'segmentos_direita':[{k:round(v,4) if isinstance(v,float) else v for k,v in m.items() if k!='score'} for m in right[:3]],
         'hipotese_par_compativel':False,'motivo':'SEGMENTO_EM_AMBOS_LADOS_NAO_ENCONTRADO'}
    if not left or not right:return out
    pairs=[]
    for l in left:
        for r in right:
            a1=math.atan(l['slope']); a2=math.atan(r['slope'])
            overlap=min(l['x_max'],r['x_max'])-max(l['x_min'],r['x_min'])
            if overlap<0.25:continue
            if abs(a1-a2)>cfg.max_parallel_error_rad:continue
            xmid=(max(l['x_min'],r['x_min'])+min(l['x_max'],r['x_max']))/2.0
            width=(l['slope']*xmid+l['offset']-r['slope']*xmid-r['offset'])/math.sqrt(1+((l['slope']+r['slope'])/2)**2)
            if cfg.min_width_m<=width<=cfg.max_width_m:
                pairs.append((min(l['score'],r['score']),width,overlap,a1,a2))
    if not pairs:
        out['motivo']='PARES_SEM_SOBREPOSICAO_OU_SEM_LARGURA_E_PARALELISMO';return out
    pairs.sort(reverse=True)
    _,width,overlap,a1,a2=pairs[0]
    out['hipotese_par_compativel']=True
    out['largura_hipotese_m']=round(width,3)
    out['sobreposicao_longitudinal_m']=round(overlap,3)
    out['diferenca_angular_graus']=round(abs(math.degrees(a1-a2)),2)
    out['motivo']='PAR_GEOMETRICO_POSSIVEL_SEM_VALIDAR_FOLGA_LOCAL'
    return out
