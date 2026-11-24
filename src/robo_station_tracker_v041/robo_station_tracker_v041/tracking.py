"""Associacao por sobreposicao de retas, memoria com evidencias e sem comandos de movimento."""
from dataclasses import dataclass,field
import math
import statistics


def axis_error(a,b):
    return abs((a-b+math.pi/2)%math.pi-math.pi/2)


def dot(p,d):return p[0]*d[0]+p[1]*d[1]


def to_xy(p):return (float(p[0]),float(p[1]))


@dataclass
class Observation:
    p0:tuple
    p1:tuple
    depths:list=field(default_factory=list)
    wall_continuation:bool=False

    def __post_init__(self):
        self.p0=to_xy(self.p0);self.p1=to_xy(self.p1)
        self.width=math.dist(self.p0,self.p1)
        self.axis=math.atan2(self.p1[1]-self.p0[1],self.p1[0]-self.p0[0])%math.pi
        self.center=((self.p0[0]+self.p1[0])/2,(self.p0[1]+self.p1[1])/2)


def geometry_match(p0,p1,q0,q1,angle_gate=math.radians(20),offset_gate=.16,gap_gate=.14):
    """Relacao entre duas retas no mapa; tolera centros diferentes por oclusao.

    Requer eixos paralelos, pequena separacao normal e sobreposicao ou lacuna
    longitudinal pequena. Nao depende de distancia entre centros.
    """
    aa=math.atan2(p1[1]-p0[1],p1[0]-p0[0])%math.pi
    bb=math.atan2(q1[1]-q0[1],q1[0]-q0[0])%math.pi
    angle=axis_error(aa,bb)
    if angle>angle_gate:return None
    d=(math.cos(aa),math.sin(aa));n=(-d[1],d[0])
    midp=((p0[0]+p1[0])/2,(p0[1]+p1[1])/2)
    midq=((q0[0]+q1[0])/2,(q0[1]+q1[1])/2)
    off=abs(dot((midq[0]-midp[0],midq[1]-midp[1]),n))
    if off>offset_gate:return None
    a,b=sorted((dot(p0,d),dot(p1,d)))
    c,e=sorted((dot(q0,d),dot(q1,d)))
    overlap=max(0.,min(b,e)-max(a,c))
    gap=max(0.,max(a,c)-min(b,e))
    len0=b-a;len1=e-c
    # Evita reidentificar dois pedacos adjacentes de uma parede como a mesma face.
    # Sob oclusao admitimos contato entre fragmentos apenas com lacuna minima.
    if len0>=.75 and len1>=.75 and overlap<.55*min(len0,len1):
        # Duas faces completas que coincidem pouco nao sao o mesmo objeto.
        return None
    if overlap < .17*min(len0,len1):
        if gap > min(gap_gate, .045) or off > min(offset_gate,.05):return None
    cost=1.7*off/offset_gate + angle/angle_gate + gap/gap_gate - .50*overlap/max(1e-6,min(len0,len1))
    return (cost,overlap,off,angle)


@dataclass
class Track:
    id:int
    p0:tuple
    p1:tuple
    n:int
    last_seen:float
    first_seen:float
    views:list=field(default_factory=list)
    depth_views:list=field(default_factory=list)
    depths:list=field(default_factory=list)
    side_hits:int=0
    wall_hits:int=0
    suspected_wall:bool=False
    samples:list=field(default_factory=list) # historico compacto de faces medidas, sem uniao cumulativa

    @property
    def width(self):return math.dist(self.p0,self.p1)
    @property
    def axis(self):return math.atan2(self.p1[1]-self.p0[1],self.p1[0]-self.p0[0])%math.pi
    @property
    def center(self):return ((self.p0[0]+self.p1[0])/2,(self.p0[1]+self.p1[1])/2)
    @property
    def depth(self):
        if not self.depths:return None
        return sorted(self.depths)[len(self.depths)//2]


class Tracker:
    def __init__(self,angle_gate_deg=15.0,offset_gate=.10,gap_gate=.12,
                 short_age=12.,memory_age=180.,max_tracks=40,view_spacing=.23):
        self.tracks=[];self.next_id=1
        self.angle_gate=math.radians(angle_gate_deg)
        self.offset_gate=offset_gate;self.gap_gate=gap_gate
        self.short_age=short_age;self.memory_age=memory_age
        self.view_spacing=view_spacing;self.max_tracks=max_tracks

    def reset(self):
        self.tracks=[];self.next_id=1

    def _long_lived(self,t):
        return len(t.views)>=2 and t.n>=20 and len(t.depth_views)>=1

    def expire(self,now):
        self.tracks=[t for t in self.tracks if now-t.last_seen <= (self.memory_age if self._long_lived(t) else self.short_age)]

    def _add_view(self,arr,sensor):
        sensor=to_xy(sensor)
        if all(math.dist(sensor,v)>=self.view_spacing for v in arr):
            if len(arr)<40:arr.append(sensor)

    def _blend_segment(self,t,obs):
        """Estimativa robusta da face sem uniao irrestrita dos extremos.

        O algoritmo anterior guardava min/max de TODAS as observacoes. Leituras
        de uma parede proxima podiam alongar a face a cada passagem. Aqui usamos
        a mediana das faces suficientemente completas; observacoes parciais
        auxiliam a associacao, mas nao esticam automaticamente a face.

        Esta estimativa NAO produz confirmacao de bancada e NAO inventa lados.
        """
        t.samples.append((obs.p0,obs.p1))
        t.samples=t.samples[-80:]
        d=(math.cos(t.axis),math.sin(t.axis)); n=(-d[1],d[0])
        base_z=dot(t.center,n)
        supported=[]
        for a,b in t.samples:
            aa=math.atan2(b[1]-a[1],b[0]-a[0])%math.pi
            if axis_error(t.axis,aa)>math.radians(12):continue
            u,v=sorted((dot(a,d),dot(b,d)))
            z=.5*(dot(a,n)+dot(b,n))
            if abs(z-base_z)>.09:continue
            supported.append((u,v,z))
        if not supported:return
        complete=[v for v in supported if .75 <=v[1]-v[0]<=1.05]
        if complete:
            positions=[.5*(lo+hi) for lo,hi,z in complete]
            widths=[hi-lo for lo,hi,z in complete]
            center=statistics.median(positions)
            length=statistics.median(widths)
            z=statistics.median(v[2] for v in complete)
        else:
            # Uma face parcial nao deve crescer pela soma de trechos vizinhos.
            # Mantemos a maior observacao isolada consistente, nao a uniao.
            chosen=max(supported,key=lambda v:v[1]-v[0])
            center=.5*(chosen[0]+chosen[1]);length=chosen[1]-chosen[0];z=chosen[2]
        t.p0=(d[0]*(center-length/2)+n[0]*z,d[1]*(center-length/2)+n[1]*z)
        t.p1=(d[0]*(center+length/2)+n[0]*z,d[1]*(center+length/2)+n[1]*z)

    def _observe(self,t,obs,sensor,now):
        t.n+=1;t.last_seen=now
        self._blend_segment(t,obs)
        self._add_view(t.views,sensor)
        if obs.depths:
            t.side_hits+=1
            t.depths.extend(float(x) for x in obs.depths)
            t.depths=t.depths[-80:]
            self._add_view(t.depth_views,sensor)
        if obs.wall_continuation:t.wall_hits+=1
        if t.wall_hits>=4 and t.wall_hits*3>t.n:t.suspected_wall=True

    def update(self,observations,sensor_pose,now):
        self.expire(now)
        obs=list(observations)
        options=[]
        for i,o in enumerate(obs):
            for j,t in enumerate(self.tracks):
                m=geometry_match(t.p0,t.p1,o.p0,o.p1,self.angle_gate,self.offset_gate,self.gap_gate)
                if m is not None:options.append((m[0],i,j))
        used_obs=set();used_tracks=set()
        for _,i,j in sorted(options):
            if i in used_obs or j in used_tracks:continue
            used_obs.add(i);used_tracks.add(j)
            self._observe(self.tracks[j],obs[i],sensor_pose,now)
        for i,o in enumerate(obs):
            if i in used_obs:continue
            if len(self.tracks)>=self.max_tracks:break
            t=Track(self.next_id,o.p0,o.p1,1,now,now,samples=[(o.p0,o.p1)])
            self._add_view(t.views,sensor_pose)
            if o.depths:
                t.side_hits=1;t.depths=list(o.depths);self._add_view(t.depth_views,sensor_pose)
            if o.wall_continuation:t.wall_hits=1
            self.next_id+=1;self.tracks.append(t)
        self._merge_duplicates()
        return self.tracks

    def _merge_duplicates(self):
        # Apenas faces muito coincidentes sao fundidas; nunca usar extensao
        # cumulativa como prova de que duas partes pertencem a uma bancada.
        changed=True
        while changed:
            changed=False
            for i in range(len(self.tracks)):
                if changed:break
                for j in range(i+1,len(self.tracks)):
                    a,b=self.tracks[i],self.tracks[j]
                    m=geometry_match(a.p0,a.p1,b.p0,b.p1,math.radians(8),.05,.025)
                    if m is None or m[1]<.60*min(a.width,b.width):continue
                    d=(math.cos(a.axis),math.sin(a.axis))
                    vals=[dot(p,d) for p in (a.p0,a.p1,b.p0,b.p1)]
                    if max(vals)-min(vals)>1.055:continue
                    if a.id>b.id:a,b=b,a
                    a.samples=(a.samples+b.samples)[-80:]
                    # Reestimativa robusta a partir das amostras, sem adicionar
                    # o segmento de b como outra observacao independente.
                    if b.samples:
                        self._blend_segment(a,Observation(*b.samples[-1]))
                    a.n+=b.n;a.last_seen=max(a.last_seen,b.last_seen)
                    a.first_seen=min(a.first_seen,b.first_seen)
                    a.side_hits+=b.side_hits;a.depths=(a.depths+b.depths)[-80:]
                    a.wall_hits+=b.wall_hits;a.suspected_wall|=b.suspected_wall
                    for v in b.views:self._add_view(a.views,v)
                    for v in b.depth_views:self._add_view(a.depth_views,v)
                    self.tracks=[t for t in self.tracks if t.id!=b.id]
                    changed=True;break

    def status(self,t,now):
        age=max(0.,now-t.last_seen)
        if t.suspected_wall:state='SUSPEITA_DE_PAREDE_OU_MESCLAGEM'
        elif t.side_hits>=3 and len(t.depth_views)>=2 and len(t.views)>=2:
            state='GEOMETRIA_COMPATIVEL_NAO_VERIFICADA'
        elif t.side_hits:state='LATERAL_POSSIVEL_NAO_VERIFICADA'
        elif t.width<.75:state='FACE_PARCIAL'
        else:state='CANDIDATA_GEOMETRIA_INCOMPLETA'
        if age>1.:state='OCULTA_'+state
        return state
