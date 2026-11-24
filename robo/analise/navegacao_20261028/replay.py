import os
import pickle, sys, math, bisect
from datetime import datetime
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ferramentas'))
import zonas_velocidade as Z
d = pickle.load(open(sys.argv[1], 'rb'))
mo, ob = d['mo'], d['ob']; mot=[r[0] for r in mo]; obt=[r[0] for r in ob]
def comp(a,b):
    x,y,th=a; return (x+b[0]*math.cos(th)-b[1]*math.sin(th), y+b[0]*math.sin(th)+b[1]*math.cos(th), th+b[2])
def pose(t):
    i=bisect.bisect_right(mot,t)-1; j=bisect.bisect_right(obt,t)-1; return comp(mo[i][1:], ob[j][1:])
plans=[p for p in d['plans'] if p[1]=='/plan']; pt=[p[0] for p in plans]
sc=d['scans']; st=[s[0] for s in sc]
cn=d['cmds']['/cmd_vel_nav']; ct=[c[0] for c in cn]
day = datetime.fromtimestamp(mot[0]).strftime('%Y-%m-%d ')
ts=lambda s: datetime.strptime(day+s, '%Y-%m-%d %H:%M:%S.%f').timestamp()
legs=[l.split() for l in open(sys.argv[2])]
cfg=dict(Z.CFG_PADRAO)
TOT_R=TOT=GAN=0
for nome,a,b in legs:
    t0,t1=ts(a),ts(b); h=Z.Histerese(0.6); t=t0; nr=0; n=0; ganho=0; motivos={}
    trechos=[]; cur=None
    while t<t1:
        i=bisect.bisect_right(pt,t)-1; cam=plans[i][2] if i>=0 and t-plans[i][0]<2.5 else None
        j=bisect.bisect_right(st,t)-1; pts=sc[j][1] if j>=0 and t-sc[j][0]<0.6 else None
        pts=[(float(x),float(y)) for x,y in pts] if pts is not None else None
        quer,mtv,med=Z.avaliar(cam,pose(t),pts,cfg); r=h.passo(t,quer)
        k=bisect.bisect_right(ct,t)-1; v=math.hypot(cn[k][1],cn[k][2]) if k>=0 else 0
        if r:
            nr+=1
            if v>0.07: ganho+=0.2*(1-0.08/0.13)
        else: motivos[mtv.split(' (')[0]]=motivos.get(mtv.split(' (')[0],0)+1
        n+=1; t+=0.2
    TOT+=n*0.2; TOT_R+=nr*0.2; GAN+=ganho
    top=sorted(motivos.items(), key=lambda kv:-kv[1])[:3]
    print('%-12s %5.1f s  RAPIDO %4.1f s (%2.0f%%)  ganho ~%.1f s | NORMAL por: %s' % (nome,n*0.2,nr*0.2,100*nr/max(1,n),ganho, ', '.join('%s %.0fs'%(m,c*0.2) for m,c in top)))
print('TOTAL nav %.0f s, RAPIDO %.0f s, ganho estimado ~%.0f s' % (TOT,TOT_R,GAN))
