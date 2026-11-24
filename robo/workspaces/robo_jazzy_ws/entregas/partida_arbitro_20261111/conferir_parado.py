"""Amostragem somente leitura de TF/IMU/comandos, sem publicar nada."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from tf2_ros import Buffer,TransformListener
from rcl_interfaces.srv import GetParameters
rclpy.init();n=rclpy.create_node('diagnostico_parado_supervisor');b=Buffer();l=TransformListener(b,n)
d={'cmd_vel':[],'cmd_vel_nav':[],'tf':[],'imu':[],'slam':None}
def cmd(topico,m):d[topico].append([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z])
for topico in ('cmd_vel','cmd_vel_nav'):n.create_subscription(TwistStamped,'/'+topico,lambda m,t=topico:cmd(t,m),20)
def yaw(q):return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
n.create_subscription(Imu,'/imu/data',lambda m:d['imu'].append(yaw(m.orientation)),qos_profile_sensor_data)
c=n.create_client(GetParameters,'/slam_toolbox/get_parameters');f=None
fim=time.monotonic()+10;proxima=0
while rclpy.ok() and time.monotonic()<fim:
 rclpy.spin_once(n,timeout_sec=.1)
 if f is None and c.service_is_ready():
  req=GetParameters.Request();req.names=['mode','map_file_name'];f=c.call_async(req)
 if f is not None and f.done() and d['slam'] is None:d['slam']=[v.string_value for v in f.result().values]
 if time.monotonic()>=proxima:
  proxima=time.monotonic()+.5
  try:
   a=b.lookup_transform('map','base_link',Time());o=b.lookup_transform('map','odom',Time())
   d['tf'].append(dict(x=a.transform.translation.x,y=a.transform.translation.y,yaw=yaw(a.transform.rotation),map_odom_yaw=yaw(o.transform.rotation)))
  except Exception:pass
res={k:{'amostras':len(d[k]),'nao_zero':sum(any(abs(v)>1e-5 for v in cmd) for cmd in d[k])} for k in ('cmd_vel','cmd_vel_nav')}
res['slam']=d['slam'];res['tf']=d['tf'];res['imu_amostras']=len(d['imu'])
if d['imu']:res['imu_yaw_min_max']=[min(d['imu']),max(d['imu'])]
p=Path(__file__).parent/'diagnostico_parado.json';p.write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))
n.destroy_node();rclpy.try_shutdown()
