"""Verificacao de geometria: hipoteses, jamais classificacao definitiva."""
import math
from .tracking import axis_error


def side_evidence(front,segments,depth_min=.25,depth_max=.50,corner_tol=.10,angle_tol_deg=20):
    a,b,_,_=front
    dx,dy=b[0]-a[0],b[1]-a[1]
    l=math.hypot(dx,dy)
    if l<=1.e-5:return []
    nx,ny=-dy/l,dx/l
    cx,cy=(a[0]+b[0])/2,(a[1]+b[1])/2
    if nx*cx+ny*cy<0:nx,ny=-nx,-ny
    hits=[]
    for corner in (a,b):
        for other in segments:
            if other is front:continue
            p0,p1,_,_=other
            side=math.dist(p0,p1)
            if not depth_min<=side<=depth_max:continue
            heading0=math.atan2(dy,dx)
            heading1=math.atan2(p1[1]-p0[1],p1[0]-p0[0])
            if abs(axis_error(heading0,heading1)-math.pi/2)>math.radians(angle_tol_deg):continue
            if math.dist(p0,corner)<corner_tol:base,tip=p0,p1
            elif math.dist(p1,corner)<corner_tol:base,tip=p1,p0
            else:continue
            projected=(tip[0]-base[0])*nx+(tip[1]-base[1])*ny
            if projected<.75*depth_min:continue
            hits.append(side)
            break
    return hits


def continuation(front,segments,angle_tol_deg=12,gap_tol=.16,min_total=1.22):
    """Testa se face e outra reta formam prolongamento colinear extenso.

    Evidencia apenas de parede: posicao da bancada junto a parede e oclusoes
    ainda podem produzir falso positivo. Nunca autoriza comando de movimento.
    """
    a,b,_,_=front
    d=(b[0]-a[0],b[1]-a[1]);length=math.hypot(*d)
    if length<1.e-5:return False
    ux,uy=d[0]/length,d[1]/length
    norm=(-uy,ux)
    midpoint=((a[0]+b[0])/2,(a[1]+b[1])/2)
    for other in segments:
        if other is front:continue
        c,e,_,_=other
        ll=math.dist(c,e)
        if ll<.15:continue
        v=(e[0]-c[0],e[1]-c[1])
        err=axis_error(math.atan2(d[1],d[0]),math.atan2(v[1],v[0]))
        if err>math.radians(angle_tol_deg):continue
        # Uma parede deslocada para tras nao e prolongamento da face.
        displacement=abs((c[0]-midpoint[0])*norm[0]+(c[1]-midpoint[1])*norm[1])
        if displacement>.08:continue
        positions=[(p[0]-midpoint[0])*ux+(p[1]-midpoint[1])*uy for p in (a,b,c,e)]
        ext=max(positions)-min(positions)
        if ext<min_total:continue
        base=sorted(positions[:2]);otherp=sorted(positions[2:])
        gap=max(0.,max(base[0],otherp[0])-min(base[1],otherp[1]))
        if gap<=gap_tol:return True
    return False
