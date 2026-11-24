"""Audita origem DDS por publisher_gid. Somente leitura."""
import json,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TwistStamped
rclpy.init();n=rclpy.create_node('auditar_fontes_comando');mensagens=[];fontes={}
topicos=('/cmd_vel_nav_base','/cmd_vel_ctrl','/cmd_vel_nav','/cmd_vel_nav_especial')
def callback(m,info,t):
 v=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]
 mensagens.append(dict(topico=t,gid=bytes(info['publisher_gid'] if isinstance(info,dict) else info.publisher_gid).hex(),stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,nao_zero=any(abs(x)>1e-3 for x in v),v=v))
for t in topicos:
 def receber(m,info,topic=t):callback(m,info,topic)
 n.create_subscription(TwistStamped,t,receber,30)
fim=time.monotonic()+12
while time.monotonic()<fim:
 rclpy.spin_once(n,timeout_sec=.1)
 for t in topicos:
  for p in n.get_publishers_info_by_topic(t):fontes[bytes(p.endpoint_gid).hex()]=dict(no=p.node_namespace.rstrip('/')+'/'+p.node_name,topico=t)
res={'fontes':fontes,'resumo':{}}
for t in topicos:
 for gid,f in fontes.items():
  if f['topico']==t:
   ms=[m for m in mensagens if m['gid']==gid and m['topico']==t]
   res['resumo'][t+' '+f['no']]=dict(amostras=len(ms),nao_zero=sum(m['nao_zero'] for m in ms))
res['mensagens']=mensagens
out=Path(__file__).parent/'fontes_comando.json';out.write_text(json.dumps(res,indent=2));print(json.dumps(res['resumo'],indent=2));n.destroy_node();rclpy.try_shutdown()
