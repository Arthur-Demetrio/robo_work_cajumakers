import csv,json
from pathlib import Path
import numpy as np
p=Path(__file__).parent;r=p.parent/'runs/principal'
rows=list(csv.DictReader((r/'episodes.csv').open()))
summary=[]
for lo,hi in [(0,300000),(300000,700000),(700000,1100000),(1100000,1600000),(1600000,2000384)]:
 a=[x for x in rows if lo<int(x['steps_total'])<=hi]
 summary.append(dict(start=lo,end=hi,n=len(a),success=np.mean([x['success']=='True' for x in a]),collision=np.mean([x['collision']=='True' for x in a]),elapsed=np.mean([float(x['elapsed']) for x in a]),last100_success=np.mean([x['success']=='True' for x in a[-100:]])))
evals=[]
for d in sorted((r/'evaluations').iterdir(),key=lambda x:int(x.name)):
 if (d/'summary.json').exists():evals.append(dict(steps=int(d.name),summary=json.loads((d/'summary.json').read_text())))
(p/'history.json').write_text(json.dumps(dict(training=summary,evaluations=evals),indent=2))
print(json.dumps(summary,indent=2))
