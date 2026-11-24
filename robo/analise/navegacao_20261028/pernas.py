import os
import pickle, sys, math, bisect
from datetime import datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
d = pickle.load(open(sys.argv[1], 'rb'))
mo, ob = d['mo'], d['ob']
mot=[r[0] for r in mo]; obt=[r[0] for r in ob]
def comp(a,b):
    x,y,th=a; return (x+b[0]*math.cos(th)-b[1]*math.sin(th), y+b[0]*math.sin(th)+b[1]*math.cos(th), th+b[2])
def pose(t):
    i=bisect.bisect_right(mot,t)-1; j=bisect.bisect_right(obt,t)-1
    return comp(mo[i][1:], ob[j][1:])
day = datetime.fromtimestamp(mot[0]).strftime('%Y-%m-%d ')
def ts(s): return datetime.strptime(day+s, '%Y-%m-%d %H:%M:%S.%f').timestamp()
cn = d['cmds']['/cmd_vel_nav']; cf = d['cmds']['/cmd_vel']
for nome, a, b in [l.split() for l in open(sys.argv[2])]:
    t0, t1 = ts(a), ts(b)
    # trajeto real (map), amostrado em odom-pose ob a 0.1 s
    L = 0; R = 0; p0 = pose(t0); prev = p0; t = t0
    while t < t1:
        t += 0.1; p = pose(t); L += math.hypot(p[0]-prev[0], p[1]-prev[1])
        R += abs(math.atan2(math.sin(p[2]-prev[2]), math.cos(p[2]-prev[2]))); prev = p
    reta = math.hypot(prev[0]-p0[0], prev[1]-p0[1])
    sel = [c for c in cn if t0 <= c[0] <= t1]
    v = [math.hypot(c[1], c[2]) for c in sel]
    sat = sum(1 for x in v if x > 0.07) / max(1, len(v))
    gir = sum(1 for c in sel if math.hypot(c[1], c[2]) < 0.015 and abs(c[3]) > 0.01) / max(1, len(v))
    zero = sum(1 for c in sel if math.hypot(c[1], c[2]) < 0.015 and abs(c[3]) <= 0.01) / max(1, len(v))
    vm = sum(v)/max(1,len(v))
    print('%-14s %5.1f s  trajeto %.2f m (reta %.2f)  v_real %.3f m/s  giro %3.0f graus | cmd medio %.3f  sat>0.07 %3.0f%%  so_giro %3.0f%%  parado %3.0f%%' % (nome, t1-t0, L, reta, L/(t1-t0), math.degrees(R), vm, 100*sat, 100*gir, 100*zero))
