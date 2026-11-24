"""So leitura: grava /scan_raw, /scan_front, /scan_nav e /odom por N s em JSON (nenhum publisher)."""
import sys, json, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]); OUT = sys.argv[2]
rclpy.init()
n = Node('amostra_scan_leitura')
dados = {'/scan_raw': [], '/scan_front': [], '/scan_nav': [], 'odom': []}
meta = {}


def cb(top):
    def f(m):
        meta.setdefault(top, dict(amin=m.angle_min, ainc=m.angle_increment, n=len(m.ranges), frame=m.header.frame_id,
                                  rmin=m.range_min, rmax=m.range_max))
        dados[top].append([time.monotonic(), [None if not math.isfinite(r) else round(r, 4) if r == r else None
                                              for r in m.ranges],
                           [1 if (r != r) else 0 for r in m.ranges]])   # marca NaN
    return f


for t in ('/scan_raw', '/scan_front', '/scan_nav'):
    n.create_subscription(LaserScan, t, cb(t), qos_profile_sensor_data)
n.create_subscription(Odometry, '/odom', lambda m: dados['odom'].append(
    [time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x, m.twist.twist.linear.y,
     m.twist.twist.angular.z]), 20)
t0 = time.monotonic()
while time.monotonic() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
json.dump(dict(meta=meta, dados=dados), open(OUT, 'w'))
print({k: len(v) for k, v in dados.items()}, meta)
n.destroy_node()
rclpy.shutdown()
