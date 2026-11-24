from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    package_dir = get_package_share_directory("robo_navigation")

    # Arquivo de parametros dentro de config/. Ex.: config:=nav2_smac.yaml
    params_file = PathJoinSubstitution([
        package_dir,
        "config",
        LaunchConfiguration("config")
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),

        DeclareLaunchArgument(
            "config",
            default_value="nav2.yaml",
            description="Arquivo de parametros em config/ (nav2.yaml = NavFn, "
                        "nav2_smac.yaml = Smac Lattice)"
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time}
            ]
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time}
            ]
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time}
            ]
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time}
            ]
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator"
                ]
            }]
        ),
    ])