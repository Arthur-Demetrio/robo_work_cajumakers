#!/usr/bin/env bash
# Sete janelas de validacao, sem iniciar Nav2 ou enviar objetivos.
set -e
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} == --pane ]]; then
    pane=${2:-}
    case "$pane" in
        hardware) pattern='(^|/| )hardware_bridge_node( |$)'; group=base ;;
        lidar) pattern='(^|/| )scan_filter_node( |$)'; group=base ;;
        localizacao) pattern='(^|/| )command_imu_odometry_node( |$)'; group=base ;;
        mapa) pattern='(^|/| )async_slam_toolbox_node( |$)'; group=base ;;
        camera) pattern='(^|/)virtual_wall( |$)'; group=visao ;;
        tf) pattern='(^|/| )tf2_echo map base_link( |$)'; group=visao ;;
        rviz) pattern='(^|/| )rviz2( |$)'; group=visao ;;
        *) exit 2 ;;
    esac
    if pgrep -u "$(id -u)" -f "$pattern" >/dev/null; then
        echo "$pane ja esta em execucao. Instancia existente preservada."
        echo 'Esta janela fica disponivel para diagnostico.'
        export ROS_DOMAIN_ID=0
        source /opt/ros/jazzy/setup.bash
        if [[ "$group" == base ]]; then
            source "$HOME/robo_jazzy_ws/install/setup.bash"
        else
            source "$script_dir/../install/setup.bash"
        fi
        cd "$script_dir/.."
        exec bash -i
    fi
    if [[ "$group" == base ]]; then
        exec bash "$script_dir/09-base-validacao-fita.sh" --pane "$pane"
    else
        exec bash "$script_dir/08-validar-fita.sh" --pane "$pane"
    fi
fi
failed=0
for pane in hardware lidar localizacao mapa camera tf rviz; do
    if ! gnome-terminal --window --title="Validacao fita - $pane" \
        -- bash "$script_dir/10-validacao-fita-completa.sh" --pane "$pane"; then
        echo "Nao foi possivel abrir a janela: $pane" >&2
        failed=1
    fi
done
exit "$failed"
