"""Teste ROS isolado: arbitro real; heartbeat para em T; mede ate quando cmd nav2 passa."""
import subprocess, sys, time, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool
ARB = os.path.expanduser(sys.argv[1])
p = subprocess.Popen([sys.executable, ARB], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
rclpy.init(); n = Node('teste_lease')
base = n.create_publisher(TwistStamped, '/cmd_vel_nav_base', 10)
hb = n.create_publisher(TwistStamped, '/navegacao/heartbeat', 1)
saida = []
n.create_subscription(TwistStamped, '/cmd_vel_nav', lambda m: saida.append((time.monotonic(), m.twist.linear.x)), 50)
cli = n.create_client(SetBool, '/navegacao/especial')
assert cli.wait_for_service(timeout_sec=10)
f = cli.call_async(SetBool.Request(data=False)); rclpy.spin_until_future_complete(n, f, timeout_sec=3)
print('troca', f.result().message)
t0 = time.monotonic(); ultimo_hb = None
while time.monotonic() - t0 < 5.0:
    agora = time.monotonic()
    m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg()
    if agora - t0 < 2.0:
        m.header.frame_id = 'nav2'; hb.publish(m); ultimo_hb = agora
    c = TwistStamped(); c.header.stamp = n.get_clock().now().to_msg(); c.twist.linear.x = -0.1; base.publish(c)
    rclpy.spin_once(n, timeout_sec=0.0)
    end = time.monotonic() + 0.1
    while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.01)
nz = [t for t, v in saida if abs(v) > 1e-3]
print('ultimo heartbeat rel', round(ultimo_hb - t0, 3), 'ultimo nz repassado rel', round(max(nz) - t0, 3) if nz else None,
      'atraso_bloqueio', round(max(nz) - ultimo_hb, 3) if nz else None)
p.send_signal(2); p.wait(5); n.destroy_node(); rclpy.shutdown()
