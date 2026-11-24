#!/usr/bin/env bash
# Base ROS + camera metrica/Nav2 + TF + RViz em sete terminais.
set -e
script_path=$(readlink -f "${BASH_SOURCE[0]}")
script_dir=$(dirname "$script_path")
if [[ ${1:-} == --pane ]]; then
    case ${2:-} in
        hardware|lidar|localizacao|mapa|tf)
            exec bash "$script_dir/10-validacao-fita-completa.sh" --pane "$2" ;;
        nav2)
            exec bash "$script_dir/11-ativar-fita-nav2.sh" --foreground ;;
        rviz)
            exec bash "$script_dir/11-ativar-fita-nav2.sh" --rviz ;;
        *) exit 2 ;;
    esac
fi
failed=0
for pane in hardware lidar localizacao mapa tf nav2 rviz; do
    title="Fita Nav2 - $pane"
    if [[ "$pane" == nav2 ]]; then title='Fita Nav2 - camera metrica e navegacao'; fi
    if ! gnome-terminal --window --title="$title" \
        -- bash "$script_path" --pane "$pane"; then
        echo "Falha ao abrir terminal: $pane" >&2
        failed=1
    fi
done
exit "$failed"
