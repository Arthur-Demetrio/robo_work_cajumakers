"""Observer only: parameter GET, graph inspection, sensor/costmap subscriptions.

Does not send navigation goals, set parameters, publish Twist, call lifecycle changes,
clear costmaps, or activate nodes. Run in the robot's domain only for inspection.
"""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

PARAMS = {
    '/controller_server': ['FollowPath.plugin', 'FollowPath.primary_controller', 'FollowPath.sim_time',
                           'FollowPath.critics', 'controller_frequency', 'progress_checker.plugin'],
    '/global_costmap/global_costmap': ['plugins', 'obstacle_layer.plugin', 'obstacle_layer.enabled',
                                     'obstacle_layer.observation_sources', 'obstacle_layer.scan.topic',
                                     'update_frequency', 'publish_frequency', 'global_frame'],
    '/local_costmap/local_costmap': ['plugins', 'obstacle_layer.enabled',
                                   'obstacle_layer.observation_sources', 'obstacle_layer.scan.topic',
                                   'update_frequency', 'publish_frequency', 'global_frame'],
    '/bt_navigator': ['default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'],
    '/alinhador_passagem': ['rota_mapa', 'virar_caminho_graus', 'modo_curva_t'],
    '/collision_monitor': ['cmd_vel_in_topic', 'cmd_vel_out_topic', 'source_timeout'],
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()
    if not 1 <= args.duration <= 30:
        parser.error('duration must be 1..30 s')
    rclpy.init()
    node = Node('navigation_readonly_audit')
    samples = {}
    def receive(topic, msg):
        now = node.get_clock().now().nanoseconds / 1e9
        record = samples.setdefault(topic, {'count': 0, 'max_stamp_age_s': 0.0})
        record['count'] += 1
        age = now - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9
        record['max_stamp_age_s'] = max(record['max_stamp_age_s'], age)
        record['last_frame'] = msg.header.frame_id
        if hasattr(msg, 'data'):
            record['occupied_cells_last'] = sum(v >= 65 for v in msg.data)
    subscriptions = []
    for topic, typ in [('/scan_front', LaserScan), ('/scan_nav', LaserScan),
                       ('/global_costmap/costmap', OccupancyGrid), ('/local_costmap/costmap', OccupancyGrid)]:
        subscriptions.append(node.create_subscription(typ, topic, lambda m, t=topic: receive(t, m),
                                                       qos_profile_sensor_data))
    deadline = time.monotonic() + args.duration
    clients, pending, results = {}, {}, {}
    while time.monotonic() < deadline:
        for name, keys in PARAMS.items():
            if name not in clients:
                clients[name] = AsyncParameterClient(node, name)
            client = clients[name]
            if name not in pending and client.services_are_ready():
                pending[name] = client.get_parameters(keys)
        rclpy.spin_once(node, timeout_sec=0.05)
    for name, future in pending.items():
        if future.done() and future.exception() is None:
            results[name] = {k: parameter_value_to_python(v) for k, v in
                             zip(PARAMS[name], future.result().values)}
    graph = {}
    for topic in ['/cmd_vel_ctrl', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/scan_nav']:
        graph[topic] = [{'node': p.node_namespace.rstrip('/') + '/' + p.node_name,
                         'type': p.topic_type} for p in node.get_publishers_info_by_topic(topic)]
    nodes = [ns.rstrip('/') + '/' + name for name, ns in node.get_node_names_and_namespaces()]
    out = {'nodes': nodes, 'parameters': results, 'publishers': graph, 'observations': samples,
           'unanswered_nodes': [n for n in PARAMS if n not in results],
           'live_navigation_verified': '/bt_navigator' in results and '/controller_server' in results,
           'caution': 'A nonempty global costmap does not prove new scan obstacles are inserted. '
                      'Per-action behavior_tree overrides are not visible through parameter GET.'}
    args.output.write_text(json.dumps(out, indent=2, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
