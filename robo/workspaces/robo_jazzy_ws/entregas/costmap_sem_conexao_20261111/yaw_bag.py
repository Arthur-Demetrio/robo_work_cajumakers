"""Analise offline apos encerramento. Correcoes de map->odom, nao ground truth."""
from pathlib import Path
import math,json,bisect
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
b=Path(__file__).parent;r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(b/'ensaio_goal_superior'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
tipos={t.name:get_message(t.type) for t in r.get_all_topics_and_types()};imu=[];tf=[];comandos=[];goals=[]
def yaw(q):return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
def st(t):return t.sec+t.nanosec/1e9
def delta(a,b):return (b-a+180)%360-180
while r.has_next():
 top,d,rec=r.read_next()
 if top not in ('/tf','/imu/data','/cmd_vel','/rosout'):continue
 m=deserialize_message(d,tipos[top])
 if top=='/imu/data':imu.append((st(m.header.stamp),yaw(m.orientation)))
 elif top=='/tf':
  for t in m.transforms:
   if t.header.frame_id.strip('/')=='map' and t.child_frame_id.strip('/')=='odom':tf.append((st(t.header.stamp),yaw(t.transform.rotation),t.transform.translation.x,t.transform.translation.y))
 elif top=='/cmd_vel':comandos.append((st(m.header.stamp),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z))
 elif top=='/rosout' and m.name=='alinhador_passagem' and m.msg.startswith('Novo goal:'):goals.append((st(m.stamp),m.msg))
imu.sort();tf.sort();it=[t for t,y in imu]
def imu_perto(t):
 i=bisect.bisect_left(it,t);return min(imu[max(0,i-1):i+1],key=lambda p:abs(p[0]-t)) if imu else None
mudancas=[]
for a,c in zip(tf,tf[1:]):
 dy=abs(delta(a[1],c[1]));dxy=math.hypot(c[2]-a[2],c[3]-a[3])
 if dy>.001 or dxy>.001:
  u=imu_perto(a[0]);v=imu_perto(c[0]);di=abs(delta(u[1],v[1])) if u and v else None
  mudancas.append(dict(t=c[0],delta_yaw_graus=dy,delta_translacao_map_odom_m=dxy,delta_imu_entre_amostras=di,dt=c[0]-a[0]))
for ev in mudancas:
 janela=[y for t,y in imu if abs(t-ev['t'])<=.25]
 if janela:
  un=[janela[0]]
  for ya,yb in zip(janela,janela[1:]):un.append(un[-1]+delta(ya,yb))
  ev['imu_variacao_janela_05s_graus']=max(un)-min(un)
  ev['imu_amostras_janela_05s']=len(janela)
r=dict(tf_amostras=len(tf),imu_amostras=len(imu),correcoes=len(mudancas),saltos_yaw_acima3graus=sum(c['delta_yaw_graus']>3 for c in mudancas),max_delta_yaw_graus=max((c['delta_yaw_graus'] for c in mudancas),default=0),maiores_correcoes=sorted(mudancas,key=lambda c:c['delta_yaw_graus'],reverse=True)[:8],comandos_finais=len(comandos),comandos_finais_nao_zero=sum(any(abs(v)>1e-5 for v in c[1:]) for c in comandos),goals_recebidos=goals,limite='TF map->odom mede correcao do referencial, nao deslocamento fisico do robo; IMU mais proxima e comparacao por intervalo curto, nao validacao absoluta.')
(b/'yaw_bag.json').write_text(json.dumps(r,indent=2,ensure_ascii=False));print(json.dumps(r,indent=2,ensure_ascii=False))
