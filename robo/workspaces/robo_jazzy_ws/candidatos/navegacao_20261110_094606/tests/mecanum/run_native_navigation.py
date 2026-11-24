"""Repeatable native plugin A/B in an ideal in-process plant; no movement publication."""
import argparse
import json
import hashlib
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--horizon', action='store_true')
    parser.add_argument('--tuning', action='store_true')
    parser.add_argument('--profiles', nargs='+', choices=['shim', 'dwb', 'early', 'predictive'])
    parser.add_argument('--scenarios', nargs='+')
    args = parser.parse_args()
    output = ROOT / 'review/antecipacao'
    binary = ROOT / 'build_candidate/native_navigation/native_navigation'
    profiles = [('shim', 'nav2_params_shim_reference.yaml'), ('dwb', 'nav2_params_dwb_candidate.yaml'), ('early', 'nav2_params_early_candidate.yaml'), ('predictive', 'nav2_params_predictive_candidate.yaml')]
    if args.profiles:
        profiles = [(name, filename) for name, filename in profiles if name in args.profiles]
    cases = [(s, False) for s in ('open', 'lateral', 'curve', 'near', 'corner', 'final_yaw')]
    cases += [('obstacle', False), ('obstacle', True), ('dynamic', True)]
    if args.scenarios:
        cases = [(name, replan) for name, replan in cases if name in args.scenarios]
    existing = output / 'native_comparison.json'
    records = ([r for r in json.loads(existing.read_text()) if r['profile'] not in args.profiles or (args.scenarios and r['scenario'] not in args.scenarios)]
               if args.profiles and existing.exists() else [])
    for label, filename in profiles:
        for scenario, replan in cases:
            key = f'{label}_{scenario}' + ('_replan' if replan else '')
            target = output / f'{key}.json'
            target.unlink(missing_ok=True)
            command = [str(binary), '--ros-args', '--params-file', str(ROOT / 'src/robo_navigation/config' / filename),
                       '-p', 'experiment.scenario:=' + scenario,
                       '-p', 'experiment.output:=' + str(target),
                       '-p', 'experiment.replanning:=' + str(replan).lower()]
            with (output / f'{key}.log').open('w') as log:
                result = subprocess.run(command, env={**os.environ, 'ROS_DOMAIN_ID': '166',
                                                       'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF'},
                                        stdout=log, stderr=subprocess.STDOUT, timeout=45)
            record = {'profile': label, 'returncode': result.returncode}
            if target.exists() and target.stat().st_size:
                record.update(json.loads(target.read_text()))
            records.append(record)
            print(key, result.returncode, record.get('complete'), flush=True)
    if args.horizon:
        for horizon in (0.8, 1.6):
            key = f'dwb_horizon_{horizon}'
            target = output / f'{key}.json'
            target.unlink(missing_ok=True)
            with (output / f'{key}.log').open('w') as log:
                result = subprocess.run([str(binary), '--ros-args', '--params-file',
                                         str(ROOT / 'src/robo_navigation/config/nav2_params_dwb_candidate.yaml'),
                                         '-p', 'FollowPath.sim_time:=' + str(horizon),
                                         '-p', 'experiment.scenario:=obstacle',
                                         '-p', 'experiment.output:=' + str(target)],
                                        env={**os.environ, 'ROS_DOMAIN_ID': '166',
                                             'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF'},
                                        stdout=log, stderr=subprocess.STDOUT, timeout=45)
            record = {'profile': key, 'returncode': result.returncode}
            if target.exists() and target.stat().st_size:
                record.update(json.loads(target.read_text()))
            records.append(record)
            print(key, result.returncode, record.get('complete'), flush=True)
    if args.tuning:
        for parameter, value in [('PathAlign.scale', 0.0), ('GoalAlign.scale', 0.0),
                                 ('PathDist.scale', 12.0), ('ObstacleFootprint.scale', 1.0)]:
            key = 'dwb_tuning_' + parameter.replace('.', '_')
            target = output / f'{key}.json'
            target.unlink(missing_ok=True)
            with (output / f'{key}.log').open('w') as log:
                result = subprocess.run([str(binary), '--ros-args', '--params-file',
                                         str(ROOT / 'src/robo_navigation/config/nav2_params_dwb_candidate.yaml'),
                                         '-p', 'FollowPath.' + parameter + ':=' + str(value),
                                         '-p', 'experiment.scenario:=obstacle',
                                         '-p', 'experiment.replanning:=true',
                                         '-p', 'experiment.output:=' + str(target)],
                                        env={**os.environ, 'ROS_DOMAIN_ID': '166',
                                             'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF'},
                                        stdout=log, stderr=subprocess.STDOUT, timeout=45)
            record = {'profile': key, 'parameter': parameter, 'value': value, 'returncode': result.returncode}
            if target.exists() and target.stat().st_size:
                record.update(json.loads(target.read_text()))
            records.append(record)
            print(key, result.returncode, record.get('complete'), flush=True)
    (output / 'native_comparison.json').write_text(json.dumps(records, indent=2))
    (output / 'native_provenance.json').write_text(json.dumps({
        'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'configs': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                    (ROOT/'src/robo_navigation/config').glob('nav2_params_*candidate.yaml')},
        'model': 'Ideal target-twist integration dt=.1, synthetic map and inflation. No BT, smoother, CM, encoder error, scan delay, or hardware in this loop.'}, indent=2))
    if any(r['returncode'] for r in records):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
