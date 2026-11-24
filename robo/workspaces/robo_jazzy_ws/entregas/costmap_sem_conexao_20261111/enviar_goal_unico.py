"""Envio unico AUTORIZADO pelo usuario, via interface supervisionada. Sem retentativa."""
from pathlib import Path
import json,time
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import LaserScan,Imu
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.msg import Log
from std_srvs.srv import Trigger
rclpy.init();n=rclpy.create_node('ensaio_goal_superior_unico');pub=n.create_publisher(PoseStamped,'/goal_pose_alinhado',10)
sensores={};eventos=[];estados=[];comandos=[];enviado=False;terminal=False;stamp_envio=0.0;resultado=None
for topic,tipo in (('/scan_front',LaserScan),('/imu/data',Imu)):
 def sensor(m,t=topic):sensores[t]=(time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec/1e9)
 n.create_subscription(tipo,topic,sensor,qos_profile_sensor_data)
def estado(m):
 for s in m.status_list:
  st=s.goal_info.stamp.sec+s.goal_info.stamp.nanosec/1e9
  if enviado and st>=stamp_envio-.05:estados.append(dict(id=bytes(s.goal_info.goal_id.uuid).hex(),status=s.status,stamp=st))
n.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',estado,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
def log(m):
 st=m.stamp.sec+m.stamp.nanosec/1e9
 if enviado and st>=stamp_envio and m.name in ('alinhador_passagem','planner_server','bt_navigator','destravar'):eventos.append(dict(stamp=st,no=m.name,texto=m.msg))
n.create_subscription(Log,'/rosout',log,100)
n.create_subscription(TwistStamped,'/cmd_vel',lambda m:comandos.append([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),20)
cancel=n.create_client(Trigger,'/navegacao/cancelar')
try:
 limite=time.monotonic()+8
 while time.monotonic()<limite:
  rclpy.spin_once(n,timeout_sec=.1)
  if len(sensores)==2 and pub.get_subscription_count()>=1:break
 subs=n.get_subscriptions_info_by_topic('/goal_pose_alinhado')
 if len(subs)!=1 or subs[0].node_name!='alinhador_passagem':raise RuntimeError('interface supervisionada nao e unica')
 agora_ros=n.get_clock().now().nanoseconds/1e9
 if len(sensores)!=2 or any(time.monotonic()-t>.5 or not 0<=agora_ros-st<.5 for t,st in sensores.values()):raise RuntimeError('scan/IMU nao recentes')
 m=PoseStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.position.x=1.72;m.pose.position.y=1.81;m.pose.orientation.w=1.0
 stamp_envio=n.get_clock().now().nanoseconds/1e9;enviado=True;pub.publish(m)
 print('GOAL UNICO ENVIADO: x=1.72 y=1.81 yaw=0 via /goal_pose_alinhado',flush=True)
 limite=time.monotonic()+90
 while time.monotonic()<limite:
  rclpy.spin_once(n,timeout_sec=.1)
  if any(s['status'] in (4,5,6) for s in estados):
   resultado=next(s for s in reversed(estados) if s['status'] in (4,5,6));terminal=True;break
  if any(time.monotonic()-t>.8 for t,st in sensores.values()):raise RuntimeError('sensor ficou obsoleto durante ensaio')
 if not terminal:raise RuntimeError('timeout de 90s')
 # Receber mensagens de resultado/logs que chegam logo apos status terminal.
 fim=time.monotonic()+2
 while time.monotonic()<fim:rclpy.spin_once(n,timeout_sec=.1)
except Exception as exc:
 resultado={'erro':str(exc)}
finally:
 if enviado and not terminal:
  if cancel.wait_for_service(timeout_sec=2):
   f=cancel.call_async(Trigger.Request());fim=time.monotonic()+5
   while not f.done() and time.monotonic()<fim:rclpy.spin_once(n,timeout_sec=.1)
  fim=time.monotonic()+2
  while time.monotonic()<fim:rclpy.spin_once(n,timeout_sec=.1)
 d=dict(goal=[1.72,1.81,0],interface='/goal_pose_alinhado',envios=int(enviado),resultado=resultado,estados=estados,eventos=eventos,comandos_finais_observados=len(comandos),comandos_finais_nao_zero=sum(any(abs(v)>1e-5 for v in c) for c in comandos))
 out=Path(__file__).parent/'resultado_goal.json';out.write_text(json.dumps(d,indent=2,ensure_ascii=False));print(json.dumps(d,indent=2,ensure_ascii=False));n.destroy_node();rclpy.try_shutdown()
