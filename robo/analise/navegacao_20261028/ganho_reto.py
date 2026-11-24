import pickle, sys, math, bisect
d = pickle.load(open(sys.argv[1], 'rb'))
mo, ob = d['mo'], d['ob']; mot=[r[0] for r in mo]; obt=[r[0] for r in ob]
def comp(a,b):
    x,y,th=a; return (x+b[0]*math.cos(th)-b[1]*math.sin(th), y+b[0]*math.sin(th)+b[1]*math.cos(th), th+b[2])
def pose(t):
    i=bisect.bisect_right(mot,t)-1; j=bisect.bisect_right(obt,t)-1; return comp(mo[i][1:], ob[j][1:])
def opose(t):
    j=bisect.bisect_right(obt,t)-1; return ob[j][1:]
cf = d['cmds']['/cmd_vel']
# janelas de 1 s com cmd final saturado e sem giro
res=[]
i=0
while i < len(cf):
    t0=cf[i][0]; win=[c for c in cf[i:i+40] if c[0]-t0<=1.0]
    if len(win)>=8 and all(math.hypot(c[1],c[2])>0.07 and abs(c[3])<0.03 for c in win):
        t1=win[-1][0]; a=pose(t0); b=pose(t1); oa=opose(t0); obb=opose(t1)
        vm=math.hypot(b[0]-a[0],b[1]-a[1])/(t1-t0); vo=math.hypot(obb[0]-oa[0],obb[1]-oa[1])/(t1-t0)
        vc=sum(math.hypot(c[1],c[2]) for c in win)/len(win)
        res.append((vc,vm,vo)); i+=len(win)
    else: i+=1
print(len(res),'janelas')
import statistics as st
print('cmd %.3f  real(map) %.3f  odom %.3f' % tuple(st.median([r[k] for r in res]) for k in range(3)))
