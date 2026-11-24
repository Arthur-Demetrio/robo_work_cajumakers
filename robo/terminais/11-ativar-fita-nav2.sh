#!/usr/bin/env bash
# Troca a previa por visao metrica e inicia Nav2 operacional com filtro de fita.
# Mapa, hardware, lidar e localizacao permanecem nos terminais anteriores.
set -e
script_path=$(readlink -f "${BASH_SOURCE[0]}")
workspace_path=$(dirname "$(dirname "$script_path")")
if [[ ${1:-} == --rviz ]]; then
    source /opt/ros/jazzy/setup.bash
    source "$HOME/robo_jazzy_ws/install/setup.bash"
    export ROS_DOMAIN_ID=0
    rviz2 -d "$workspace_path/src/robo_virtual_wall/config/tape_validation.rviz" || true
    exec bash -i
fi
if [[ ${1:-} != --foreground ]]; then
    exec bash "$workspace_path/terminais/12-fita-nav2-completo.sh"
fi
cd "$workspace_path"
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
source "$HOME/robo_jazzy_ws/install/setup.bash"
# Somente o pacote de visao: preserva o launch, BT e parametros operacionais.
source "$workspace_path/install/robo_virtual_wall/share/robo_virtual_wall/package.bash"
if pgrep -u "$(id -u)" -f '(^|/| )(controller_server|planner_server|tape_keepout)( |$)' >/dev/null; then
    echo 'Nav2 ou tape_keepout ja esta rodando. Encerre a instancia anterior antes de ativar.'
    exec bash -i
fi
# Encerra apenas a previa da fita, sem tocar nos nos da base ROS.
mapfile -t preview_pids < <(pgrep -u "$(id -u)" -f '(^|/)virtual_wall( |$)' || true)
if (( ${#preview_pids[@]} )); then
    kill -INT "${preview_pids[@]}" || true
    for attempt in 1 2 3 4 5; do
        if ! pgrep -u "$(id -u)" -f '(^|/)virtual_wall( |$)' >/dev/null; then break; fi
        sleep 1
    done
    if pgrep -u "$(id -u)" -f '(^|/)virtual_wall( |$)' >/dev/null; then
        echo 'A camera anterior nao encerrou. Feche-a com Ctrl+C e execute novamente.'
        exec bash -i
    fi
fi
ros2 launch robo_virtual_wall operational_tape.launch.py \
    homography_file:="$workspace_path/src/robo_virtual_wall/calibration/front_base_link_validated.json" \
    preview_homography_file:="$workspace_path/src/robo_virtual_wall/calibration/front_preview_20261106_repositioned.json" \
    filters_file:="$workspace_path/src/robo_navigation/config/virtual_wall_filters.yaml" \
    params_file:="$HOME/robo_jazzy_ws/install/robo_navigation/share/robo_navigation/config/nav2_params.yaml" \
    navigation_launch:="$HOME/robo_jazzy_ws/install/robo_navigation/share/robo_navigation/launch/navigation.launch.py" || true
exec bash -i
