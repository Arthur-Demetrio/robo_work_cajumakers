"""OFFLINE ONLY. No hardware bridge, station aligner or legacy recovery is started.

Use a separate ROS domain. All velocity topics are confined to /mecanum_candidate.
The A/B profiles differ only in the controller YAML. Integrated changes are opt-in.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def setup(context):
    if os.environ.get('ROS_DOMAIN_ID') != '166':
        raise RuntimeError('Offline candidate requires ROS_DOMAIN_ID=166; do not join robot domain')
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in ('candidate', 'shim', 'global', 'dwb', 'early', 'predictive', 'integrated', 'integrated-predictive', 'stable'):
        raise RuntimeError('unknown offline candidate profile')
    share = get_package_share_directory('robo_navigation')
    integrated = profile in ('candidate', 'integrated', 'integrated-predictive', 'stable')
    filename = ('shim_reference.yaml' if profile == 'shim' else
                'global_shim_candidate.yaml' if profile == 'global' else
                'predictive_candidate.yaml' if profile in ('predictive', 'integrated-predictive', 'stable') else
                'early_candidate.yaml' if profile == 'early' else
                'dwb_candidate.yaml')
    params = os.path.join(share, 'config', 'nav2_params.yaml' if profile == 'candidate' else 'nav2_params_' + filename)
    bt = os.path.join(share, 'behavior_trees', 'navigate_to_pose_' +
                      ('candidate.xml' if profile == 'candidate' else 'stable_candidate.xml' if profile == 'stable' else 'mecanum_candidate.xml' if integrated else 'ab_candidate.xml'))
    ns = 'mecanum_candidate'
    common = [RewrittenYaml(source_file=params, root_key=ns,
                            param_rewrites={}, convert_types=True), {'use_sim_time': True}]
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                 ('/scan_nav', 'scan_nav'), ('/scan_front', 'scan_front'),
                 ('/odom', 'odom'), ('/bloqueio_virtual', 'bloqueio_virtual')]
    nodes = []
    lifecycle = []
    for package, executable in [('nav2_controller', 'controller_server'),
                                ('nav2_planner', 'planner_server'),
                                ('nav2_smoother', 'smoother_server'),
                                ('nav2_bt_navigator', 'bt_navigator'),
                                ('nav2_velocity_smoother', 'velocity_smoother'),
                                ('nav2_collision_monitor', 'collision_monitor')]:
        overrides = {}
        remaps = list(tf_remaps)
        if executable == 'bt_navigator':
            overrides['default_nav_to_pose_bt_xml'] = bt
        if executable == 'controller_server':
            remaps.append(('cmd_vel', 'cmd_vel_ctrl' if integrated else 'cmd_vel_nav'))
        if executable == 'velocity_smoother':
            remaps.append(('cmd_vel', 'cmd_vel_nav'))
        nodes.append(Node(package=package, executable=executable, name=executable,
                          namespace=ns, parameters=common + [overrides], remappings=remaps,
                          output='screen'))
        lifecycle.append(executable)
    if integrated:
        nodes.append(Node(package='robo_recovery', executable='supervised_recovery',
                          namespace=ns, parameters=[{'use_sim_time': True}],
                          remappings=tf_remaps, output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_candidate', namespace=ns,
                      parameters=[{'use_sim_time': True, 'autostart': False,
                                   'node_names': lifecycle}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='candidate', choices=['candidate', 'shim', 'global', 'dwb', 'early', 'predictive', 'integrated', 'integrated-predictive', 'stable']),
        OpaqueFunction(function=setup),
    ])
