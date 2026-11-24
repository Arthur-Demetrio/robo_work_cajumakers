#!/usr/bin/env bash
# Abre somente visão e ferramentas de inspeção. Não inicia controle de motores.
set -e
script_path=$(readlink -f "${BASH_SOURCE[0]}")
workspace_path=$(dirname "$(dirname "$script_path")")
if [[ ${1:-} == --pane ]]; then
    cd "$workspace_path"
    export ROS_DOMAIN_ID=0
    source /opt/ros/jazzy/setup.bash
    source "$workspace_path/install/setup.bash"
    case ${2:-} in
        camera)
            ros2 run robo_virtual_wall virtual_wall --ros-args \
                -p show_windows:=true -p roi_top:=0.0 \
                -p width:=640 -p height:=480 \
                -p preview_homography_file:="$workspace_path/src/robo_virtual_wall/calibration/front_preview_20261106_repositioned.json" || true
            ;;
        tf)
            echo 'Inspecao da pose do robo no mapa; Ctrl+C volta ao terminal.'
            ros2 run tf2_ros tf2_echo map base_link || true
            ;;
        rviz)
            rviz2 || true
            ;;
    esac
    exec bash -i
fi
gnome-terminal --window --title='Fita - camera 640x480' \
    -- bash "$script_path" --pane camera
gnome-terminal --window --title='Fita - TF map para base_link' \
    -- bash "$script_path" --pane tf
gnome-terminal --window --title='Fita - RViz' \
    -- bash "$script_path" --pane rviz
