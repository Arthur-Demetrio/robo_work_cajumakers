"""Auditoria somente leitura: nao publica goals, velocidades, parametros ou limpeza."""
from pathlib import Path
import time,json,math
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped
from tf2_ros import Buffer,TransformListener
rclpy.init();n=rclpy.create_node('auditar_scans_costmap');buf=Buffer();tl=TransformListener(buf,n);scans={};foot=[]
for topic in ('/scan_front','/scan_nav'):
 n.create_subscription(LaserScan,topic,lambda m,t=topic:scans.update({t:m}),qos_profile_sensor_data)
n.create_subscription(PolygonStamped,'/global_costmap/published_footprint',lambda m:foot.append(m),10)
c=n.create_client(GetParameters,'/global_costmap/global_costmap/get_parameters');f=None;params=['footprint','footprint_padding','robot_radius','inflation_layer.inflation_radius','inflation_layer.cost_scaling_factor','obstacle_layer.scan.topic','static_layer.map_topic']
end=time.monotonic()+8
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if f is None and c.service_is_ready():
  req=GetParameters.Request();req.names=params;f=c.call_async(req)
out=Path(__file__).parent;meta={};arrays={}
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
for topic,m in scans.items():
 name=topic.strip('/');arrays[name+'_ranges']=np.array(m.ranges)
 try:t=buf.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp));temporal=True
 except Exception:t=buf.lookup_transform('map',m.header.frame_id,Time());temporal=False
 a=np.arange(len(m.ranges))*m.angle_increment+m.angle_min+yaw(t.transform.rotation);r=np.array(m.ranges);ok=np.isfinite(r)&(r>=m.range_min)&(r<=m.range_max)
 points=np.column_stack((t.transform.translation.x+r[ok]*np.cos(a[ok]),t.transform.translation.y+r[ok]*np.sin(a[ok])))
 arrays[name+'_map']=points
 meta[topic]=dict(frame=m.header.frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,tf_no_stamp=temporal,range_min=m.range_min,range_max=m.range_max,pontos=len(points),tf=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)])
if foot:meta['footprint']=dict(frame=foot[-1].header.frame_id,points=[[v.x,v.y] for v in foot[-1].polygon.points])
if f and f.done():
 def value(v):return {'1':v.bool_value,'2':v.integer_value,'3':v.double_value,'4':v.string_value}.get(str(v.type),str(v))
 meta['parametros']=dict(zip(params,[value(v) for v in f.result().values]))
np.savez_compressed(out/'scans.npz',**arrays);(out/'scans.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2));n.destroy_node();rclpy.try_shutdown()
