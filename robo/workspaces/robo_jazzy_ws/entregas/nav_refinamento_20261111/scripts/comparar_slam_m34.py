#!/usr/bin/env python3
"""Compara TF map->odom ja gravados; sem ROS, sem movimentacao.
Saltos da origem odom NAO equivalem ao deslocamento do robo; nao ha ground truth.
"""
import csv, math, json
from pathlib import Path
base=Path(__file__).resolve().parents[1]/'slam_replay'
result={}
for modo in ('mapping','localizacao'):
    folder=base/('m34_'+modo)
    rows=[tuple(float(r[k]) for k in ('stamp','x','y','yaw')) for r in csv.DictReader((folder/'map_odom.csv').open())]
    # descarta transformacao inicial identidade antes da primeira correcao do mapa
    first=next(i for i,r in enumerate(rows) if abs(r[1])+abs(r[2])+abs(r[3])>1e-8)
    rows=rows[first:]
    updates=[]
    for a,b in zip(rows,rows[1:]):
        d=math.hypot(b[1]-a[1],b[2]-a[2]); yaw=abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3])))
        if d>1e-8 or yaw>1e-8: updates.append((d,math.degrees(yaw)))
    def stats(v):
        s=sorted(v)
        return {'max':max(s,default=0),'p95':s[int(.95*(len(s)-1))] if s else 0}
    log=(folder/'slam.log').read_text()
    result[modo]={'amostras':len(rows),'duracao_s':rows[-1][0]-rows[0][0], 'correcoes':len(updates),
      'delta_origem_odom_m':stats([d for d,y in updates]),'delta_yaw_graus':stats([y for d,y in updates]),
      'saltos_yaw_acima_5graus':sum(y>5 for d,y in updates), 'avisos_scan_descartado':log.count('Message Filter dropping message'),
      'fim_presente':(folder/'fim').exists()}
(base/'comparacao_m34.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False))
