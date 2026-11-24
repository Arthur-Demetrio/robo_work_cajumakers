"""Selecao geometrica de WS e rastreamento de uma unica face; sem ROS."""
import math

TOL_ANG_REF = math.radians(20)   # FACE_ANG20 (09/11): era 12 graus
JANELA_FALHA = .5   # FALHA_SCAN (09/11): era .3 s; a 5,9 Hz tolera 1 scan ruim entre 2 bons
# JANELA_LARGA (09/11, missao30): a WS_1 estava inteira a vista (0,88 m, y -0,15..+0,74) mas a ponta
# passava da janela de 0,70 m e o centro (30 cm de lado) passava dos 18 cm: recusava a mesa certa.
# Agora o robo reconhece a mesa ate ~30 cm de lado e desliza ate o centro. A largura (0,60-0,98 m),
# a distancia (15 cm) e o angulo continuam filtrando paredes.
Y_LIM_FACE = .95       # era .70
TOL_CENTRO_REF = .35   # era .18


def ang(x):
    return math.atan2(math.sin(x), math.cos(x))


def transformar_ponto(p, origem, destino):
    """Ponto em origem base -> mundo odom -> destino base."""
    c,s=math.cos(origem[2]),math.sin(origem[2])
    wx,wy=origem[0]+c*p[0]-s*p[1],origem[1]+s*p[0]+c*p[1]
    dx,dy=wx-destino[0],wy-destino[1]
    c,s=math.cos(destino[2]),math.sin(destino[2])
    return c*dx+s*dy,-s*dx+c*dy


def pose_mapa_no_scan(mapa_agora, odom_agora, odom_scan):
    dx,dy=transformar_ponto((0,0),odom_scan,odom_agora)
    c,s=math.cos(mapa_agora[2]),math.sin(mapa_agora[2])
    return (mapa_agora[0]+c*dx-s*dy,mapa_agora[1]+s*dx+c*dy,
            ang(mapa_agora[2]+odom_scan[2]-odom_agora[2]))


def referencia_ws(face, pose):
    if pose is None or not all(math.isfinite(v) for v in pose):
        return None
    if not all(math.isfinite(face[k]) for k in ('x','y','yaw')):
        return None
    x,y=transformar_ponto((face['x'],face['y']),(0,0,0),pose)
    a=ang(face['yaw']-pose[2])
    if abs(a)>math.radians(25):
        return None
    width=face.get('largura_medida',0.8)
    if not math.isfinite(width) or not .6<=width<=1.0:
        return None
    return dict(dist=x+math.tan(a)*y,ang=a,
                centro=-math.sin(a)*x+math.cos(a)*y,span=width)


def _ajustar(points, y_lim, tol):
    if len(points)<6:return None
    for _ in range(2):
        m=len(points)
        if m<6:return None
        my=sum(p[1] for p in points)/m;mx=sum(p[0] for p in points)/m
        sy=sum((p[1]-my)**2 for p in points)
        if sy<1e-4:return None
        b=sum((p[1]-my)*(p[0]-mx) for p in points)/sy
        a=mx-b*my
        points=[p for p in points if abs(p[0]-a-b*p[1])<=tol]
    if len(points)<6:return None
    lo=min(p[1] for p in points);hi=max(p[1] for p in points)
    span=hi-lo
    if span<.25:return None
    yc=(lo+hi)/2;xc=a+b*yc
    centro=(yc+xc*b)/math.sqrt(1+b*b) if .60<=span<=.98 and lo>-y_lim+.05 and hi<y_lim-.05 else None
    return dict(dist=a,ang=-math.atan(b),centro=centro,span=span,n=len(points),
                residual=max(abs(p[0]-a-b*p[1]) for p in points))


def candidatos_face(pts,x_min=.30,x_max=1.30,y_lim=None,tol=.02):
    """Ate quatro planos; separa segmentos colineares por lacunas >10 cm."""
    if y_lim is None:y_lim=Y_LIM_FACE
    remaining=[p for p in pts if len(p)==2 and all(math.isfinite(v) for v in p)
               and x_min<=p[0]<=x_max and abs(p[1])<=y_lim]
    result=[]
    for _ in range(4):
        if len(remaining)<6:break
        best=None
        for k in range(-50,51):
            th=math.radians(k*.5);c,s=math.cos(th),math.sin(th)
            offsets=sorted((x*c-y*s,(x,y)) for x,y in remaining)
            j=0
            for i in range(len(offsets)):
                while offsets[i][0]-offsets[j][0]>.03:j+=1
                n=i-j+1
                if best is None or n>best[0] or (n==best[0] and offsets[j][0]<best[2]):
                    best=(n,th,offsets[j][0],(offsets[i][0]+offsets[j][0])/2)
        if best[0]<6:break
        c,s=math.cos(best[1]),math.sin(best[1])
        line=[p for p in remaining if abs(p[0]*c-p[1]*s-best[3])<=tol]
        if len(line)<6:break
        taken=set(line);remaining=[p for p in remaining if p not in taken]
        line.sort(key=lambda p:p[1])
        segments=[[]]
        for p in line:
            if segments[-1] and math.dist(p,segments[-1][-1])>.10:segments.append([])
            segments[-1].append(p)
        for segment in segments:
            fit=_ajustar(segment,y_lim,tol)
            if fit is not None:result.append(fit)
    return result


def selecionar_face(pts, referencia=None, **kwargs):
    candidates=candidatos_face(pts,**kwargs)
    if referencia is None:
        return max(candidates,key=lambda m:(m['n'],-m['dist'])) if candidates else None
    ranked=[]
    for m in candidates:
        if m['centro'] is None:continue
        dd=abs(m['dist']-referencia['dist']); da=abs(ang(m['ang']-referencia['ang']))
        dc=abs(m['centro']-referencia['centro']);dw=abs(m['span']-referencia['span'])
        # FACE_ANG20 (09/11): era 12 graus. Missao25: LiDAR via a WS_2 limpa, mas o
        # yaw do mapa diferia 8-16 graus da face real (cadastro + deriva de lado).
        if dd>.15 or da>TOL_ANG_REF or dc>TOL_CENTRO_REF or dw>.12:continue
        ranked.append((dd/.15+da/TOL_ANG_REF+dc/TOL_CENTRO_REF+dw/.12,m))
    ranked.sort(key=lambda v:v[0])
    # Alternativas geometricamente equivalentes nao autorizam escolher por densidade.
    if len(ranked)>1 and ranked[1][0]-ranked[0][0]<.35:return None
    return ranked[0][1] if ranked else None


def motivo_face(pts, referencia, **kwargs):
    """MOTIVO_FACE (09/11): por que selecionar_face recusou, com os numeros (so para o log)."""
    if referencia is None:
        return 'sem referencia (pose no mapa/TF)'
    cands=candidatos_face(pts,**kwargs)
    inteiras=[m for m in cands if m['centro'] is not None]
    if not inteiras:
        return 'nenhuma face com as 2 pontas (%d segmentos: %s)' % (len(cands), ', '.join(
            'd%.2f w%.2f' % (m['dist'],m['span']) for m in cands[:4]) or 'nenhum')
    partes=[];ok=[]
    for m in inteiras:
        dd=abs(m['dist']-referencia['dist']); da=abs(ang(m['ang']-referencia['ang']))
        dc=abs(m['centro']-referencia['centro']);dw=abs(m['span']-referencia['span'])
        estoura=[n for n,v,l in (('dist',dd,.15),('ang',da,TOL_ANG_REF),('centro',dc,TOL_CENTRO_REF),('larg',dw,.12)) if v>l]
        partes.append('d%.2f c%+.2f a%+.0fg w%.2f%s' % (m['dist'],m['centro'],math.degrees(m['ang']),m['span'],
                      (' FORA:'+'/'.join(estoura)) if estoura else ''))
        if not estoura:ok.append(m)
    pre='ambigua (%d faces validas)' % len(ok) if len(ok)>1 else 'fora da referencia'
    return '%s | ref d%.2f c%+.2f a%+.0fg | %s' % (pre,referencia['dist'],referencia['centro'],
                                                 math.degrees(referencia['ang']),'; '.join(partes[:3]))


def projetar_face(m,old,new):
    # Ponto medido no centro da face e direcao normal, ambos compensados pelo SE(2).
    a=m['ang'];c,s=math.cos(a),math.sin(a)
    d=m['dist']*c;t=m['centro']
    p=transformar_ponto((d*c-t*s,d*s+t*c),old,new)
    a=ang(a+old[2]-new[2]);c,s=math.cos(a),math.sin(a)
    if c<.5:return None
    return dict(dist=(p[0]*c+p[1]*s)/c,ang=a,centro=-p[0]*s+p[1]*c)


def acompanhar_face_parcial(pts, referencia):
    """Fase 2: mede o plano perto; o centro vem da face completa projetada pela odom.

    Nunca serve para reconhecer uma WS. Exige retornos atuais perto do plano
    previsto e cobertura lateral suficiente para medir sua orientacao.
    """
    if not .365 <= referencia['dist'] <= .515:
        return None
    b = -math.tan(referencia['ang'])
    points = [p for p in pts if len(p) == 2 and all(math.isfinite(v) for v in p)
              and .30 <= p[0] <= .60 and abs(p[1]) <= Y_LIM_FACE
              and abs(p[0] - referencia['dist'] - b*p[1]) <= .04]
    if len(points) < 12:
        return None
    m = _ajustar(points, Y_LIM_FACE, .025)
    if (m is None or m['n'] < 12 or
            abs(m['dist'] - referencia['dist']) > .05 or
            abs(ang(m['ang'] - referencia['ang'])) > math.radians(8)):
        return None
    m.update(centro=referencia['centro'], span=referencia['span'], parcial=True)
    return m


class RastreioFaceWS:
    def __init__(self, face):
        if (not isinstance(face,dict) or
                any(not isinstance(face.get(k), (int,float)) or not math.isfinite(face[k])
                    for k in ('x','y','yaw')) or
                not isinstance(face.get('largura_medida',.8),(int,float)) or
                not .6<=face.get('largura_medida',.8)<=1.0):
            raise ValueError('referencia WS invalida')
        self.face=dict(face);self.ultima=None;self.motivo=''
        self.reconhecida = False

    def medir(self,pts,t,odom,pose_mapa):
        if (not math.isfinite(t) or len(odom)!=3 or
                not all(math.isfinite(v) for v in odom)):
            self.ultima=None;self.motivo='tempo ou odometria invalida';return None
        ref=referencia_ws(self.face,pose_mapa)
        # TRAVA_ODOM (09/11, missao27): o mapa so escolhe a face na 1a vez. Travado nela, a
        # referencia e a face anterior projetada pela odometria: um salto do SLAM (+8,6 graus e
        # 30 cm em map->odom, robo parado em frente a WS_2) nao derruba mais o encosto.
        if ref is not None and self.ultima is not None and 0<=t-self.ultima[0]<=JANELA_FALHA:
            pred=projetar_face(self.ultima[1],self.ultima[2],odom)
            if pred is not None:
                ref=dict(pred,span=self.ultima[1]['span'])
        m=selecionar_face(pts,ref) if ref is not None else None
        if (m is None and self.reconhecida and self.ultima is not None
                and 0 <= t-self.ultima[0] <= JANELA_FALHA):
            pred = projetar_face(self.ultima[1], self.ultima[2], odom)
            if pred is not None:
                m = acompanhar_face_parcial(pts, dict(pred, span=self.ultima[1]['span']))
        if m is None:
            # FALHA_SCAN (09/11, missao29): 31-38% dos raios voltam inf e um scan isolado parte a
            # face em pedacos. Nao esquece a face travada por um scan so: o proximo segue pela odometria.
            if self.ultima is not None and not 0<=t-self.ultima[0]<=JANELA_FALHA:
                self.ultima=None
            self.motivo='face ausente, ambigua ou fora da referencia WS: '+motivo_face(pts,ref)   # MOTIVO_FACE
            return None
        old=self.ultima
        self.ultima=(t,m,odom)
        if old is None:
            self.reconhecida = False
            self.motivo='aguardando segunda observacao coerente';return None
        if not .03<=t-old[0]<=JANELA_FALHA:
            self.motivo='observacao repetida/antiga';return None
        predicted=projetar_face(old[1],old[2],odom)
        if (predicted is None or abs(predicted['dist']-m['dist'])>.06
            or abs(predicted['centro']-m['centro'])>.06
            or abs(ang(predicted['ang']-m['ang']))>math.radians(5)):
            self.ultima=None;self.motivo='salto de referencia geometrica';return None
        self.reconhecida = True
        self.motivo='face WS acompanhada (parcial)' if m.get('parcial') else 'face WS consistente'
        return m


def obstaculo_antes_da_face(pts,med,para_choque=.365):
    """Um retorno entre para-choque e WS no corredor frontal basta para parar."""
    if med is None:return None
    b=-math.tan(med['ang'])
    gaps=[x-para_choque for x,y in pts if math.isfinite(x) and math.isfinite(y)
          and abs(y)<=.335 and x>para_choque and x<med['dist']+b*y-.05]
    return min(gaps) if gaps else None
