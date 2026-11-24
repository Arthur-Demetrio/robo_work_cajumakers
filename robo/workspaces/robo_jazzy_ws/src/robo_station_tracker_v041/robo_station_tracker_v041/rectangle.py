"""Relacao entre tracks perpendiculares: hipotese, NUNCA reconhecimento confirmado.

Essa camada nao substitui os segmentos brutos e nao registra estacoes.
Usa retas JA observadas como possiveis dois lados de uma mesma caixa.
"""
import math
from .tracking import axis_error,dot


def _contact(front,side,corner,normal,corner_tolerance=.14):
    # Retorna o contato da lateral com a face e o vetor que vai para dentro
    # da hipotese. Uma extrapolacao de reta nao vale como contato medido.
    a,b=side.p0,side.p1
    touch,tip=(a,b) if math.dist(a,corner)<=math.dist(b,corner) else (b,a)
    distance=math.dist(touch,corner)
    if distance>corner_tolerance:return None
    v=(tip[0]-touch[0],tip[1]-touch[1])
    depth=math.hypot(*v)
    if not .23<=depth<=.56:return None
    normal_component=dot(v,normal)
    if abs(normal_component)<.20:return None
    return touch,tip,normal_component,depth,distance


def rectangle_hypothesis(front,other_tracks,now,age_limit=180.,
                         front_min=.70,front_max=1.15,width_min=.70,width_max=1.07):
    """Procura duas laterais ortogonais em cantos OPOSTOS da mesma face.

    Checa que as pontas das laterais caminham para o mesmo semiplano da face.
    Retorna estimativa independente de largura por distancia entre contatos.
    Nao utiliza esse resultado como confirmacao automatica de bancada.
    """
    if not front_min<=front.width<=front_max:return None
    d=(math.cos(front.axis),math.sin(front.axis));n=(-d[1],d[0])
    corners=(front.p0,front.p1)
    candidates=[[],[]]
    for side in other_tracks:
        if side.id==front.id or now-side.last_seen>age_limit:continue
        if not .23<=side.width<=.56:continue
        if abs(axis_error(front.axis,side.axis)-math.pi/2)>math.radians(20):continue
        for end in range(2):
            touch=_contact(front,side,corners[end],n)
            if touch is not None:
                candidates[end].append((side,touch))
    ranked=[]
    for a,hit0 in candidates[0]:
        for b,hit1 in candidates[1]:
            if a.id==b.id:continue
            if hit0[2]*hit1[2]<=0:continue  # pontas opostas -> nao e caixa
            xa=dot(hit0[0],d);xb=dot(hit1[0],d)
            dist=abs(xa-xb)
            if not width_min<=dist<=width_max:continue
            # Distancia entre contatos do lado correto, media dos comprimentos
            # observados, e idade das observacoes. Nada foi inferido por mapa.
            ages=(max(0.,now-a.last_seen),max(0.,now-b.last_seen))
            score=hit0[4]+hit1[4]+.10*sum(ages)/max(age_limit,1.)
            ranked.append((score,a,b,dist,(hit0[3]+hit1[3])/2,ages))
    if not ranked:return None
    _,a,b,dist,depth,ages=min(ranked,key=lambda x:x[0])
    return {
        'tracks_laterais':[f'TRACK_{a.id:02d}',f'TRACK_{b.id:02d}'],
        'largura_entre_laterais_m':round(dist,3),
        'comprimento_medio_laterais_m':round(depth,3),
        'idades_laterais_s':[round(v,2) for v in ages],
        'tipo':'HIPOTESE_RETANGULO_DUAS_LATERAIS_NAO_VERIFICADA',
        'aviso':'Segmentos geometricamente compatíveis; nao confirma identidade nem ausencia de parede'
    }
