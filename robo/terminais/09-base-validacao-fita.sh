#!/usr/bin/env bash
# Abre a base ROS necessaria para gerar map -> odom -> base_link.
# Nao inicia Nav2, goals, teleop ou rotinas de alinhamento.
set -e
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} == --pane ]]; then
    case ${2:-} in
        hardware) bash "$script_dir/01-hardware.sh" || true ;;
        lidar) bash "$script_dir/02-lidar.sh" || true ;;
        localizacao) bash "$script_dir/03-localizacao.sh" || true ;;
        mapa) bash "$script_dir/04-mapeamento.sh" || true ;;
        *) exit 2 ;;
    esac
    exec bash -i
fi
for pane in hardware lidar localizacao mapa; do
    gnome-terminal --window --title="Base fita - $pane" \
        -- bash "$script_dir/09-base-validacao-fita.sh" --pane "$pane"
done
