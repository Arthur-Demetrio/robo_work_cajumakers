"""Inicia somente os componentes Nav2 usados pelo robo durante o SLAM."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    principal = LaunchConfiguration('nav2_principal')
    saida_base = PythonExpression(["'cmd_vel_nav_base' if '", principal, "'.lower() == 'true' else 'cmd_vel_nav'"])
    saida_especial = PythonExpression(["'/cmd_vel_nav_especial' if '", principal, "'.lower() == 'true' else '/cmd_vel_nav'"])
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    modo_curva_t = LaunchConfiguration("modo_curva_t")

    navigation_share = get_package_share_directory("robo_navigation")
    default_params_file = os.path.join(
        navigation_share, "config", "nav2_params.yaml"
    )

    default_nav_to_pose_bt_xml = os.path.join(
        navigation_share,
        "behavior_trees",
        "navigate_to_pose_smooth.xml",
    )

    # O docking_server nao e iniciado: o robo nao possui rotina de docagem.
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    common_parameters = [
        params_file,
        {"use_sim_time": use_sim_time},
    ]
    common_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]
    motion_output_remappings = common_remappings + [
        ("cmd_vel", saida_base),
    ]

    progressive_spin_node = Node(
        package="robo_progressive_spin",
        executable="progressive_spin_node",
        name="progressive_spin",
        remappings=[("cmd_vel", saida_base), ("/cmd_vel_nav", saida_base)],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    mecanum_escape_node = Node(
        package="robo_recovery",
        executable="mecanum_escape_node",
        name="mecanum_escape_node",
        remappings=[("cmd_vel", saida_base), ("/cmd_vel_nav", saida_base)],
        output="screen",
    )

    lifecycle_manager_delayed = RegisterEventHandler(
        OnProcessStart(
            target_action=mecanum_escape_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        Node(
                            package="nav2_lifecycle_manager",
                            executable="lifecycle_manager",
                            name="lifecycle_manager_navigation",
                            output="screen",
                            parameters=[
                                {
                                    "use_sim_time": use_sim_time,
                                    "autostart": autostart,
                                    "node_names": lifecycle_nodes,
                                }
                            ],
                        ),
                    ],
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('nav2_principal', default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument('bt_xml', default_value=PythonExpression(["'", os.path.join(get_package_share_directory('robo_navigation'), 'behavior_trees', 'navigate_to_pose_principal.xml'), "' if '", principal, "'.lower() == 'true' else '", default_nav_to_pose_bt_xml, "'"])),
            ExecuteProcess(cmd=['python3', os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/arbitro_nav_principal.py')],
                           condition=IfCondition(principal), output='screen'),
            DeclareLaunchArgument("limite_vel_angular_real_t", default_value="-1.0"),
            DeclareLaunchArgument("limite_vel_linear_real_t", default_value="-1.0"),
            DeclareLaunchArgument("modo_curva_t", default_value="lateral",
                                  choices=["lateral"]),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Usar o relogio de simulacao",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Ativar automaticamente os nos de ciclo de vida",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PythonExpression(["'", os.path.join(get_package_share_directory('robo_navigation'), 'config', 'nav2_params_principal.yaml'), "' if '", principal, "'.lower() == 'true' else '", default_params_file, "'"]),
                description="Arquivo de parametros do Nav2",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common_parameters,
                remappings=common_remappings + [("cmd_vel", "cmd_vel_ctrl")],  # SUAVIZADOR_NAV: era /cmd_vel_nav
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=common_parameters,
                remappings=common_remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common_parameters,
                remappings=common_remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common_parameters,
                remappings=motion_output_remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=common_parameters + [
                    {
                        "default_nav_to_pose_bt_xml":
                            LaunchConfiguration("bt_xml"),
                    }
                ],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=common_parameters,
                remappings=common_remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=common_parameters,
                # PRIORIDADE_LATERAL_WS_20261109: caminho lateral original; giro suspenso.
                remappings=common_remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=common_parameters + [{
                    "stop_pub_timeout": ParameterValue(PythonExpression([
                        "86400.0 if '", modo_curva_t, "' == 'giro_frontal' else 2.0"]), value_type=float),
                    "SweptTurnStop.enabled": ParameterValue(
                        PythonExpression(["'", modo_curva_t, "' == 'giro_frontal'"]), value_type=bool)}],
                # Parametros fazem /cmd_vel_smoothed -> /cmd_vel.
                remappings=common_remappings,
            ),
            ExecuteProcess(  # NAV_TUDO: filtro do corpo: /scan_front -> /scan_nav
                cmd=["python3", os.path.expanduser(
                    "~/robo_jazzy_ws/src/robo_navigation/scripts/filtro_corpo_scan.py")],
                output="screen",
            ),
            ExecuteProcess(  # NAV_TUDO: recuperacao destravar (arvore v3)
                cmd=["python3", os.path.expanduser(
                    "~/robo_jazzy_ws/src/robo_navigation/scripts/destravar_node.py"), "--ros-args", "-r", ["/cmd_vel_nav:=", saida_base]],
                output="screen",
            ),
            ExecuteProcess(  # NAV_TUDO: alinhador: passagens, mesas, START/FINISH
                cmd=["python3", os.path.expanduser(
                    "~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py"),
                     "--ros-args", "-p", ["nav2_principal:=", principal],
                     "-p", "insistir_curva:=0", "-p", "curva_segue_nav2:=false",
                     "-r", ["/cmd_vel_nav:=", saida_especial],
                     "-p", ["modo_curva_t:=", modo_curva_t],
                     "-p", ["limite_vel_angular_real_t:=", LaunchConfiguration("limite_vel_angular_real_t")],
                     "-p", ["limite_vel_linear_real_t:=", LaunchConfiguration("limite_vel_linear_real_t")]],
                output="screen",
            ),
            ExecuteProcess(  # SUAVIZADOR_NAV: /cmd_vel_ctrl -> /cmd_vel_nav
                cmd=["python3", os.path.expanduser(
                    "~/robo_jazzy_ws/src/robo_navigation/scripts/suavizador_nav.py"), "--ros-args", "-p", ["preservar_eixos_zero:=", principal], "-r", ["/cmd_vel_nav:=", saida_base]],
                output="screen",
            ),

            progressive_spin_node,
            mecanum_escape_node,
            lifecycle_manager_delayed,
        ]
    )
