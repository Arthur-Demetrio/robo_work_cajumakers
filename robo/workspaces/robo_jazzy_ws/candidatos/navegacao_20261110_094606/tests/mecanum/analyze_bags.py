"""Read-only bag statistics; never replays a topic or derives physical success from odom."""
import argparse
import json
import math
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def analyze(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    selected = {name: get_message(typ) for name, typ in types.items()
                if name in ('/cmd_vel', '/cmd_vel_nav', '/odom', '/collision_monitor_state', '/rosout')}
    counts, previous, duration, zeros, rotations = {}, {}, {}, {}, {}
    stops = 0
    logs = []
    odom_first = odom_last = None
    for _ in iter(int, 1):
        if not reader.has_next():
            break
        topic, data, stamp = reader.read_next()
        if topic not in selected:
            continue
        msg = deserialize_message(data, selected[topic])
        counts[topic] = counts.get(topic, 0) + 1
        if topic.startswith('/cmd_vel'):
            v = msg.twist
            value = (v.linear.x, v.linear.y, v.angular.z)
            if topic in previous:
                t0, last = previous[topic]
                dt = min((stamp - t0) / 1e9, 0.3)  # hold only until watchdog timeout
                duration[topic] = duration.get(topic, 0) + dt
                if max(map(abs, last)) < 0.001:
                    zeros[topic] = zeros.get(topic, 0) + dt
                if math.hypot(*last[:2]) < 0.005 and abs(last[2]) > 0.01:
                    rotations[topic] = rotations.get(topic, 0) + dt
            previous[topic] = stamp, value
        elif topic == '/odom':
            position = msg.pose.pose.position
            current = [position.x, position.y]
            odom_first = current if odom_first is None else odom_first
            odom_last = current
        elif topic == '/collision_monitor_state':
            stops += msg.action_type == 1
        elif topic == '/rosout' and msg.level >= 30 and len(logs) < 25:
            logs.append({'node': msg.name, 'message': msg.msg})
    return {'bag': str(path), 'message_counts': counts,
            'command_covered_seconds': duration, 'zero_command_seconds': zeros,
            'pure_rotation_command_seconds': rotations, 'cm_stop_messages': stops,
            'odom_net_distance_NOT_physical': math.dist(odom_first, odom_last) if odom_first else None,
            'independent_pose_topic_available': any('observed_pose' in t for t in types),
            'warnings_sample': logs,
            'comparison': 'baseline only; no candidate bag exists; no causal or physical improvement claimed'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps([analyze(p) for p in args.bags], indent=2, ensure_ascii=False))
