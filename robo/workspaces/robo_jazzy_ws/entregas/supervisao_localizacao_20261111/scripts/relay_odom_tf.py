"""Replay: repassa do /tf_bag somente odom->base_link (map->odom vem do slam_toolbox em teste)
e grava map->odom publicado pelo slam em CSV."""
import sys, math, rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
saida = open(sys.argv[1], 'w'); saida.write('stamp,x,y,yaw\n')
rclpy.init(); n = Node('relay_odom_tf', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
pub = n.create_publisher(TFMessage, '/tf', 100)
def ao_bag(m):
    t = [x for x in m.transforms if x.header.frame_id.strip('/') == 'odom']
    if t: pub.publish(TFMessage(transforms=t))
def ao_tf(m):
    for x in m.transforms:
        if x.header.frame_id.strip('/') == 'map' and x.child_frame_id.strip('/') == 'odom':
            q = x.transform.rotation
            saida.write('%.4f,%.5f,%.5f,%.6f\n' % (x.header.stamp.sec + x.header.stamp.nanosec*1e-9, x.transform.translation.x,
                        x.transform.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
n.create_subscription(TFMessage, '/tf_bag', ao_bag, 100)
n.create_subscription(TFMessage, '/tf', ao_tf, 100)
try: rclpy.spin(n)
except Exception: pass
finally: saida.flush(); saida.close()
