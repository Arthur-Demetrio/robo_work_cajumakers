from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arquivo = LaunchConfiguration("arquivo")
    mapa = LaunchConfiguration("mapa")
    objetivo = LaunchConfiguration("objetivo")
    capacidade = LaunchConfiguration("capacidade")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arquivo", default_value="~/robo_mapas/arena_locais.yaml"
            ),
            DeclareLaunchArgument("mapa", default_value="arena"),
            DeclareLaunchArgument(
                "objetivo", default_value="",
                description="YAML de objetivo; vazio = anotacoes 'recebe' dos locais",
            ),
            DeclareLaunchArgument("capacidade", default_value="1"),
            DeclareLaunchArgument(
                "autonomo", default_value="false",
                description="true: sobe o autonomo_node, que cumpre o objetivo "
                            "sozinho ao finalizar o mapeamento",
            ),
            DeclareLaunchArgument("modo", default_value="simulado",
                                  description="simulado | nav2"),
            DeclareLaunchArgument(
                "permitir_movimento", default_value="false",
                description="true: no modo nav2 o robo anda de verdade "
                            "(como o allow_motion do robo_station_manager)",
            ),
            Node(
                package="robo_locais",
                executable="locais_node",
                name="locais",
                output="screen",
                parameters=[{"arquivo": arquivo, "mapa": mapa,
                             "arquivo_objetivo": objetivo,
                             "capacidade": capacidade}],
            ),
            Node(
                package="robo_locais",
                executable="autonomo_node",
                name="autonomo",
                output="screen",
                condition=IfCondition(LaunchConfiguration("autonomo")),
                parameters=[{"arquivo_locais": arquivo,
                             "arquivo_objetivo": objetivo,
                             "capacidade": capacidade,
                             "modo": LaunchConfiguration("modo"),
                             "permitir_movimento": ParameterValue(
                                 LaunchConfiguration("permitir_movimento"), value_type=bool)}],
            ),
        ]
    )
