"""Read-only historical map/scan evidence; no replay and no inferred physical success."""
import argparse
import bisect
import json
import math
import statistics
from pathlib import Path
import rosbag2_py
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer


def analyze(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    selected = {t: get_message(types[t]) for t in ('/scan_front', '/global_costmap/costmap',
                 '/local_costmap/costmap', '/tf', '/tf_static', '/rosout') if t in types}
    tf = Buffer(cache_time=Duration(seconds=7200))
    maps = {'/global_costmap/costmap': [], '/local_costmap/costmap': []}
    scans, last_scan, logs = [], -math.inf, []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in selected:
            continue
        msg = deserialize_message(data, selected[topic])
        if topic in ('/tf', '/tf_static'):
            for tr in msg.transforms:
                (tf.set_transform_static if topic == '/tf_static' else tf.set_transform)(tr, 'historical_bag')
        elif topic in maps:
            maps[topic].append((timestamp / 1e9, msg))
        elif topic == '/scan_front' and timestamp / 1e9 - last_scan >= 0.5:
            scans.append((timestamp / 1e9, msg))
            last_scan = timestamp / 1e9
        elif topic == '/rosout' and any(t in msg.msg for t in
                ('Using plugin', 'No valid trajectories', 'sigo a rota do mapa', 'Timed out', 'Message Filter dropping')):
            if len(logs) < 40:
                logs.append({'node': msg.name, 'message': msg.msg})
    result = {'bag': str(path), 'scans_sampled': len(scans), 'maps': {}, 'logs_sample': logs}
    for topic, grids in maps.items():
        record = {'messages': len(grids), 'tf_failures': 0, 'scan_endpoints_in_map': 0,
                  'scan_endpoints_marked': 0, 'scan_endpoints_newly_marked_vs_initial': 0,
                  'max_cells_newly_occupied_vs_initial': 0}
        if not grids:
            result['maps'][topic] = record
            continue
        times = [t for t, m in grids]
        intervals = [b-a for a,b in zip(times,times[1:])]
        record['publish_interval_median_s'] = statistics.median(intervals) if intervals else None
        record['publish_gap_max_s'] = max(intervals) if intervals else None
        initial = grids[0][1]
        def cell(grid, x, y):
            q = grid.info.origin.orientation
            angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            dx, dy = x-grid.info.origin.position.x, y-grid.info.origin.position.y
            ix = math.floor((math.cos(angle)*dx+math.sin(angle)*dy)/grid.info.resolution)
            iy = math.floor((-math.sin(angle)*dx+math.cos(angle)*dy)/grid.info.resolution)
            return grid.data[iy*grid.info.width+ix] if 0 <= ix < grid.info.width and 0 <= iy < grid.info.height else None
        for _, grid in grids:
            same = (grid.info.width == initial.info.width and grid.info.height == initial.info.height and
                    grid.info.resolution == initial.info.resolution and grid.info.origin == initial.info.origin)
            if same:
                record['max_cells_newly_occupied_vs_initial'] = max(record['max_cells_newly_occupied_vs_initial'],
                    sum(0 <= a < 20 and b >= 65 for a, b in zip(initial.data, grid.data)))
        for timestamp, scan in scans:
            index = bisect.bisect_left(times, timestamp)
            options = [i for i in (index-1, index) if 0 <= i < len(times)]
            index = min(options, key=lambda i: abs(times[i]-timestamp))
            if abs(times[index]-timestamp) > 0.75:
                continue
            grid = grids[index][1]
            try:
                tr = tf.lookup_transform(grid.header.frame_id, scan.header.frame_id, Time.from_msg(scan.header.stamp))
            except Exception:
                record['tf_failures'] += 1
                continue
            q = tr.transform.rotation
            angle = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            for i in range(0, len(scan.ranges), 5):
                r = scan.ranges[i]
                if not math.isfinite(r) or not scan.range_min <= r <= min(3.5, scan.range_max):
                    continue
                a = scan.angle_min+i*scan.angle_increment+angle
                x, y = tr.transform.translation.x+r*math.cos(a), tr.transform.translation.y+r*math.sin(a)
                v = cell(grid, x, y)
                if v is not None:
                    record['scan_endpoints_in_map'] += 1
                    record['scan_endpoints_marked'] += v >= 65
                    old = cell(initial, x, y)
                    if old is not None and 0 <= old < 20 and v >= 65:
                        record['scan_endpoints_newly_marked_vs_initial'] += 1
        result['maps'][topic] = record
    result['limits'] = ('Historical occupancy coincidences only. New cells can reflect pose/map changes; '
                        'not proof of obstacle_layer attribution, current runtime, or complete maneuver coverage.')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(analyze(args.bag), ensure_ascii=False, indent=2))
