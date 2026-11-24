from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT / 'src/robo_navigation'
SHIM_ONLY = {'primary_controller', 'angular_dist_threshold', 'angular_disengage_threshold',
             'forward_sampling_distance', 'rotate_to_heading_angular_vel', 'max_angular_accel',
             'simulate_ahead_time', 'closed_loop', 'rotate_to_heading_once'}


def test_only_wrapper_parameters_change():
    a = yaml.safe_load((NAV / 'config/nav2_params_shim_reference.yaml').read_text())
    b = yaml.safe_load((NAV / 'config/nav2_params_dwb_candidate.yaml').read_text())
    follow = a['controller_server']['ros__parameters']['FollowPath']
    follow['plugin'] = 'dwb_core::DWBLocalPlanner'
    for key in SHIM_ONLY:
        del follow[key]
    assert a == b


def test_final_goal_criteria_and_holonomic_sampling_preserved():
    p = yaml.safe_load((NAV / 'config/nav2_params_dwb_candidate.yaml').read_text())['controller_server']['ros__parameters']
    assert p['goal_checker']['yaw_goal_tolerance'] == 0.17
    assert p['FollowPath']['min_vel_y'] < 0 < p['FollowPath']['max_vel_y']
    assert 'RotateToGoal' in p['FollowPath']['critics']
    assert 'Twirling' in p['FollowPath']['critics']
    assert p['FollowPath']['min_vel_x'] == 0.0


def test_transaction_cannot_fall_through_to_unchecked_recovery():
    root = ET.parse(NAV / 'behavior_trees/navigate_to_pose_mecanum_candidate.xml').getroot()
    assert not list(root.iter('RoundRobin'))
    assert not list(root.iter('BackUp')) and not list(root.iter('Spin'))
    actions = list(root.iter('DriveOnHeading'))
    assert len(actions) == 1 and actions[0].get('server_name') == 'supervised_escape'
    assert root.find('.//RecoveryNode').get('number_of_retries') == '2'


def test_aligner_and_default_launch_untouched():
    import hashlib, json
    manifest = json.loads((ROOT/'review/source_sha256.json').read_text())
    for name in ['src/robo_navigation/scripts/alinhador_passagem.py',
                 'src/robo_navigation/launch/navigation.launch.py']:
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == manifest[name]


def test_offline_launch_domain_guard_and_namespace(monkeypatch):
    import importlib.util
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location('candidate', NAV / 'launch/mecanum_candidate_offline.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['profile'] = 'integrated'
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    import pytest
    with pytest.raises(RuntimeError, match='166'):
        module.setup(context)
    monkeypatch.setenv('ROS_DOMAIN_ID', '166')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(NAV))
    actions = module.setup(context)
    assert len(actions) == 8
    source = (NAV / 'launch/mecanum_candidate_offline.launch.py').read_text()
    assert 'RewrittenYaml(source_file=params, root_key=ns' in source
    assert "'autostart': False" in source
    assert 'alinhador_passagem.py' not in source


def test_global_normalization_preserves_effective_parameters_and_rejects_duplicates():
    original = yaml.safe_load((NAV / 'config/nav2_params_dwb_candidate.yaml').read_text())
    candidate = yaml.safe_load((NAV / 'config/nav2_params_global_unambiguous_candidate.yaml').read_text())
    assert candidate == original
    global_params = candidate['global_costmap']['global_costmap']['ros__parameters']
    assert global_params['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    assert global_params['obstacle_layer']['observation_sources'] == 'scan'
    mapping = yaml.compose((NAV / 'config/nav2_params_global_unambiguous_candidate.yaml').read_text())
    def verify(node):
        if isinstance(node, yaml.MappingNode):
            keys = [k.value for k, v in node.value]
            assert len(keys) == len(set(keys))
            for k, v in node.value:
                verify(v)
        elif isinstance(node, yaml.SequenceNode):
            for v in node.value:
                verify(v)
    verify(mapping)


def test_early_changes_only_intermediate_goal_alignment():
    base = yaml.safe_load((NAV / 'config/nav2_params_dwb_candidate.yaml').read_text())
    early = yaml.safe_load((NAV / 'config/nav2_params_early_candidate.yaml').read_text())
    base['controller_server']['ros__parameters']['FollowPath']['GoalAlign.scale'] = 0.0
    assert base == early
    follow = early['controller_server']['ros__parameters']['FollowPath']
    assert follow['RotateToGoal.scale'] == 24.0
    assert follow['PathAlign.scale'] == 24.0
    assert follow['ObstacleFootprint.scale'] == 0.1


def test_predictive_changes_only_horizon_after_goal_alignment_variant():
    early = yaml.safe_load((NAV / 'config/nav2_params_early_candidate.yaml').read_text())
    predictive = yaml.safe_load((NAV / 'config/nav2_params_predictive_candidate.yaml').read_text())
    early['controller_server']['ros__parameters']['FollowPath']['sim_time'] = 1.6
    assert early == predictive
