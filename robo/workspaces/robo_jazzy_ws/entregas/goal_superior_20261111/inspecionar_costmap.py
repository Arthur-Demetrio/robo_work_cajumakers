"""Somente leitura: snapshot do costmap/mapa e conectividade de celulas."""
from pathlib import Path
import json,time,math,collections
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init();n=rclpy.create_node('auditar_goal_superior');b=Buffer();l=TransformListener(b,n)
mapas=[]
n.create_subscription(OccupancyGrid,'/map',lambda m:mapas.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
c=n.create_client(GetCostmap,'/global_costmap/get_costmap')
if not c.wait_for_service(timeout_sec=8):raise RuntimeError('get_costmap indisponivel')
f=c.call_async(GetCostmap.Request());fim=time.monotonic()+10
while time.monotonic()<fim:
 rclpy.spin_once(n,timeout_sec=.1)
 if f.done() and mapas:
  try:tr=b.lookup_transform('map','base_link',Time());break
  except Exception:pass
else:raise RuntimeError('snapshot incompleto')
cm=f.result().map;md=cm.metadata;mp=mapas[-1];dados=np.array(cm.data,dtype=np.uint8).reshape(md.size_y,md.size_x);mapa=np.array(mp.data,dtype=np.int16).reshape(mp.info.height,mp.info.width)
out=Path(__file__).parent
np.savez_compressed(out/'snapshot.npz',costmap=dados,mapa=mapa)
inicio=(tr.transform.translation.x,tr.transform.translation.y);goal=(1.57,2.31)
def conectividade(a,res,ox,oy,start,end,permitido):
 def cel(p):return (int(math.floor((p[0]-ox)/res)),int(math.floor((p[1]-oy)/res)))
 s,g=cel(start),cel(end);h,w=a.shape
 dentro=lambda c:0<=c[0]<w and 0<=c[1]<h
 r={'inicio_cel':s,'goal_cel':g,'inicio_custo':int(a[s[1],s[0]]) if dentro(s) else None,'goal_custo':int(a[g[1],g[0]]) if dentro(g) else None}
 if not dentro(s) or not dentro(g):return dict(r,motivo='fora do grid',conectado=False)
 livre=permitido(a)
 if not livre[s[1],s[0]] or not livre[g[1],g[0]]:return dict(r,motivo='inicio ou goal bloqueado',conectado=False)
 fila=collections.deque([s]);vistos={s}
 while fila:
  x,y=fila.popleft()
  if (x,y)==g:return dict(r,conectado=True,visitadas=len(vistos))
  for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
   v=(x+dx,y+dy)
   if dentro(v) and v not in vistos and livre[v[1],v[0]]:vistos.add(v);fila.append(v)
 return dict(r,conectado=False,visitadas=len(vistos))
r={'nota':'Snapshot posterior ao goal; nao prova custos no instante da falha. BFS de celulas nao prova passagem do footprint.','pose_atual':inicio,'goal_log':goal,'costmap_metadata':dict(res=md.resolution,ox=md.origin.position.x,oy=md.origin.position.y,w=md.size_x,h=md.size_y),'mapa_metadata':dict(res=mp.info.resolution,ox=mp.info.origin.position.x,oy=mp.info.origin.position.y)}
for nome,fn in [('costmap_sem_unknown',lambda a:a<253),('costmap_com_unknown',lambda a:(a<253)|(a==255))]:r[nome]=conectividade(dados,md.resolution,md.origin.position.x,md.origin.position.y,inicio,goal,fn)
r['mapa_ocupacao']=conectividade(mapa,mp.info.resolution,mp.info.origin.position.x,mp.info.origin.position.y,inicio,goal,lambda a:(a>=0)&(a<65))
(out/'conectividade.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));n.destroy_node();rclpy.try_shutdown()
