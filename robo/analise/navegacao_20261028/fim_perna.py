import pickle, sys, math, bisect
from datetime import datetime
d = pickle.load(open(sys.argv[1], 'rb'))
mo, ob = d['mo'], d['ob']; mot=[r[0] for r in mo]; obt=[r[0] for r in ob]
def comp(a,b):
    x,y,th=a; return (x+b[0]*math.cos(th)-b[1]*math.sin(th), y+b[0]*math.sin(th)+b[1]*math.cos(th), th+b[2])
def pose(t):
    i=bisect.bisect_right(mot,t)-1; j=bisect.bisect_right(obt,t)-1; return comp(mo[i][1:], ob[j][1:])
day = datetime.fromtimestamp(mot[0]).strftime('%Y-%m-%d ')
ts=lambda s: datetime.strptime(day+s, '%Y-%m-%d %H:%M:%S.%f').timestamp()
legs=[('WS1',(2.28,0.01,-3),'09:10:45.42','09:10:51.91'),('WS3',(1.34,1.88,-177),'09:11:10.77','09:11:38.16'),
('WS2',(1.57,3.86,-171),'09:12:02.86','09:12:25.47'),('WS1b',(2.28,0.01,-3),'09:13:04.05','09:13:43.64'),
('PREP1',(0.22,-0.26,0),'09:10:17.82','09:10:30.42'),('PREPF',None,'09:14:11.49','09:14:25.25')]
for n,g,a,b in legs:
    t0,t1=ts(a),ts(b)
    if g is None: g=(*pose(t1)[:2], math.degrees(pose(t1)[2]))
    out=[]
    for r,ang in [(0.15,15),(0.10,10),(0.08,8)]:
        t=t0; hit=None
        while t<t1:
            p=pose(t); dy=abs((math.degrees(p[2])-g[2]+180)%360-180)
            if math.hypot(p[0]-g[0],p[1]-g[1])<=r and dy<=ang: hit=t;break
            t+=0.05
        out.append('%.0fcm/%d°: %s' % (r*100, ang, '%.1f s antes do fim' % (t1-hit) if hit else '-'))
    print(n, '%.1f s' % (t1-t0), ' | '.join(out))
