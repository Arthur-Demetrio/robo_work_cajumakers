import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nos(context):
    config = os.path.join(
        get_package_share_directory("robo_controle"), "config", "controle.yaml"
    )
    parametros = [config]
    # Indices de botoes/eixos do SEU controle (gerado por mapear_controle).
    # Vem depois do controle.yaml, entao sobrepoe so o que define.
    botoes = os.path.expanduser(LaunchConfiguration("botoes").perform(context))
    if botoes:
        if not os.path.isfile(botoes):
            raise RuntimeError(f"botoes:={botoes} nao existe (rode mapear_controle)")
        parametros.append(botoes)
    return [
        # Le o controle (USB ou Bluetooth) e publica /joy.
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="robo_controle",
            executable="controle_node",
            name="controle_mapeamento",
            output="screen",
            parameters=parametros,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "botoes",
                default_value="",
                description="YAML com os botoes/eixos do controle, gerado por "
                            "'ros2 run robo_controle mapear_controle'",
            ),
            OpaqueFunction(function=_nos),
        ]
    )
