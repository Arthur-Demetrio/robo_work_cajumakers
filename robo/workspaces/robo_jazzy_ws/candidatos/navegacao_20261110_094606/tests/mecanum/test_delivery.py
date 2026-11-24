import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT/'src/robo_navigation'


def test_operational_source_and_install_are_preserved():
    origin=json.loads((ROOT/'review/origem.json').read_text())
    workspace=Path(origin['workspace'])
    sources=json.loads((ROOT/'review/source_sha256.json').read_text())
    installed=json.loads((ROOT/'review/active_install_sha256.json').read_text())
    for name, digest in sources.items():
        assert hashlib.sha256((workspace/name).read_bytes()).hexdigest()==digest, name
    for name, digest in installed.items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest, name


def test_main_candidate_is_explicit_and_safe_stage_invariants():
    load=lambda name: yaml.safe_load((NAV/'config'/name).read_text())
    main=load('nav2_params.yaml'); ref=load('nav2_params_shim_reference.yaml')
    assert main==load('nav2_params_predictive_candidate.yaml')
    follow=main['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin']=='dwb_core::DWBLocalPlanner'
    assert 'primary_controller' not in follow
    original=ref['controller_server']['ros__parameters']['FollowPath']
    for k,v in original.items():
        if k.startswith(('min_vel','max_vel','min_speed','max_speed','acc_lim','decel_lim')):
            assert follow[k]==v, k
    for section in ('local_costmap','global_costmap','collision_monitor','velocity_smoother'):
        assert main[section]==ref[section], section
    tree=ET.parse(NAV/'behavior_trees/navigate_to_pose_candidate.xml')
    assert tree.find('.//RateController').get('hz')=='2.0'
    assert tree.find('.//IsPathValid').get('path')=='{remaining_path}'
    assert float(tree.find('.//TruncatePathLocal').get('distance_forward'))>0
    assert tree.find('.//SmoothPath').get('check_for_collisions')=='true'
    assert tree.find('.//RoundRobin') is None
    assert tree.find('.//ClearEntireCostmap') is None
    assert tree.find('.//BackUp') is None and tree.find('.//Spin') is None
    assert tree.find('.//DriveOnHeading').get('server_name')=='supervised_escape'


def test_yaml_has_no_duplicate_keys_in_main_candidate():
    def check(n):
        if isinstance(n,yaml.MappingNode):
            keys=[k.value for k,v in n.value]
            assert len(keys)==len(set(keys)),keys
            for k,v in n.value:check(v)
        elif isinstance(n,yaml.SequenceNode):
            for v in n.value:check(v)
    for p in (NAV/'config').glob('nav2_params_*candidate.yaml'): check(yaml.compose(p.read_text()))
    check(yaml.compose((NAV/'config/nav2_params.yaml').read_text()))


def test_candidate_launch_selects_both_files_and_is_isolated(monkeypatch):
    import importlib.util
    from launch import LaunchContext
    spec=importlib.util.spec_from_file_location('candidate_launch',NAV/'launch/navigation_candidate.launch.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    context=LaunchContext();context.launch_configurations['profile']='candidate'
    monkeypatch.setenv('ROS_DOMAIN_ID','166')
    monkeypatch.setattr(mod,'get_package_share_directory',lambda _:str(NAV))
    nodes=mod.setup(context)
    assert len(nodes)==8
    # Resolve Node parameter substitutions and inspect the actual BT override.
    bt=next(n for n in nodes if n.node_executable=='bt_navigator')
    from launch_ros.utilities import evaluate_parameters
    evaluated=evaluate_parameters(context, bt._Node__parameters)
    assert evaluated[-1]['default_nav_to_pose_bt_xml']==str(NAV/'behavior_trees/navigate_to_pose_candidate.xml')
    config=yaml.safe_load(Path(evaluated[0]).read_text())
    assert config['mecanum_candidate']['controller_server']['ros__parameters']['FollowPath']['plugin']=='dwb_core::DWBLocalPlanner'


def test_bt_ports_match_installed_plugins():
    path=ROOT/'review/plugin_ports.xml'
    assert path.exists(), 'Generate model with build_candidate/dwb_probe/bt_probe > review/plugin_ports.xml'
    model=ET.parse(path)
    ports={n.attrib['ID']:{p.attrib['name'] for p in n} for n in model.findall('.//TreeNodesModel/*')}
    tree=ET.parse(NAV/'behavior_trees/navigate_to_pose_candidate.xml')
    for node in tree.iter():
        if node.tag not in ports:continue
        assert set(node.attrib)-{'name','ID'} <= ports[node.tag], (node.tag,node.attrib,ports[node.tag])


def test_real_scan_changes_global_planner_path():
    import re
    log=(ROOT/'review/scan_replan_result.log').read_text()
    result=re.search(r'configured_source=(\S+) before_max_y=(\S+) after_max_y=(\S+) unsafe_plan_cells=(\d+)',log)
    assert result is not None
    assert result[1]=='/scan_nav'
    assert float(result[2])<0.1 and float(result[3])>0.35
    assert int(result[4])==0


def test_installed_truncation_retains_entire_forward_path():
    log=(ROOT/'review/truncate_result.log').read_text()
    assert 'remaining_points=11 first_x=1 last_x=2' in log


def test_successful_recovery_requires_a_new_collision_checked_plan():
    tree=ET.parse(NAV/'behavior_trees/navigate_to_pose_candidate.xml')
    recovery=tree.find('.//BehaviorTree/RecoveryNode/Sequence')
    assert [n.tag for n in recovery]==['Fallback','DriveOnHeading','ComputePathToPose','SmoothPath']
    assert recovery[-1].get('check_for_collisions')=='true'


def test_main_yaml_configures_real_dwb_and_has_diagonal_samples():
    import re
    result=json.loads((ROOT/'review/main_lateral.json').read_text())
    assert result['plugin']=='dwb_core::DWBLocalPlanner'
    assert result['complete'] and result['rotation_rad']==0
    log=(ROOT/'review/main_dwb_load.log').read_text()
    assert 'configured_plugin=dwb_core::DWBLocalPlanner' in log
    assert int(re.search(r'diagonal=(\d+)',log)[1])>0


def test_built_overlay_contains_the_exact_candidate_files():
    share=ROOT/'install_candidate/robo_navigation/share/robo_navigation'
    for name in ['config/nav2_params.yaml','behavior_trees/navigate_to_pose_candidate.xml','launch/navigation_candidate.launch.py']:
        assert (share/name).read_bytes()==(NAV/name).read_bytes(),name
