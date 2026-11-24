"""Read-only source audit with line references and explicit duplicate YAML keys."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml


def audit(root):
    config = root / 'src/robo_navigation/config/nav2_params.yaml'
    text = config.read_text()
    data = yaml.safe_load(text)
    duplicates = []
    def walk(node, path=''):
        if isinstance(node, yaml.MappingNode):
            seen = {}
            for key, value in node.value:
                name = key.value
                full = path + '.' + name if path else name
                if name in seen:
                    duplicates.append({'path': full, 'first_line': seen[name],
                                       'duplicate_line': key.start_mark.line + 1})
                seen[name] = key.start_mark.line + 1
                walk(value, full)
        elif isinstance(node, yaml.SequenceNode):
            for value in node.value:
                walk(value, path)
    walk(yaml.compose(text))
    evidence = {}
    files = ['src/robo_navigation/config/nav2_params.yaml',
             'src/robo_navigation/launch/navigation.launch.py',
             'src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml',
             'src/robo_navigation/scripts/alinhador_passagem.py',
             'src/robo_navigation/scripts/destravar_node.py',
             'src/robo_navigation/scripts/suavizador_nav.py',
             'src/robo_localization/scripts/command_imu_odometry_node.py']
    terms = ['RotationShimController', 'primary_controller', 'default_nav_to_pose_bt_xml',
             'navigate_to_pose_smooth.xml', 'cmd_vel', 'rota_mapa', 'sigo a rota do mapa',
             'virar_caminho_graus', 'IsPathValid', 'PathExpiringTimer', 'RoundRobin', 'server_name=',
             'check_for_collisions', 're_girar', 'visivel_tras']
    for name in files:
        content = (root / name).read_bytes()
        evidence[name] = {'sha256': hashlib.sha256(content).hexdigest(),
                          'lines': [{'line': n, 'text': line.strip()}
                                    for n, line in enumerate(content.decode().splitlines(), 1)
                                    if any(term in line for term in terms)]}
    global_params = data['global_costmap']['global_costmap']['ros__parameters']
    local_params = data['local_costmap']['local_costmap']['ros__parameters']
    return {'root': str(root), 'source_only': True,
            'live_runtime_status': 'UNVERIFIED: DDS socket/discovery unavailable in sandbox',
            'duplicate_keys': duplicates, 'global_costmap': global_params,
            'local_costmap': local_params,
            'controller': data['controller_server']['ros__parameters'], 'evidence': evidence,
            'conclusions': ['global obstacle_layer IS listed in source plugins',
                            'two source definitions of global obstacle_layer; parser-specific ambiguity',
                            'aligner has an independent static-map route override; not modified']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(audit(args.root), indent=2, ensure_ascii=False))
