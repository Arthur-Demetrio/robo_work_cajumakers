#!/bin/bash
# Uso: ~/gravar_teste.sh nome_do_teste "descrição opcional"
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
NOME=${1:-teste}
DESC=${2:-}
PASTA=~/bags/${NOME}_$(date +%Y%m%d_%H%M%S)
mkdir -p "$PASTA/ros_logs"
{ echo "teste: $NOME"; echo "descricao: $DESC"; echo "inicio: $(date '+%F %T')"; } > "$PASTA/notas.txt"

echo ">>> Salvando os parâmetros em uso..."
for n in controller_server planner_server bt_navigator behavior_server smoother_server velocity_smoother collision_monitor slam_toolbox local_costmap/local_costmap global_costmap/global_costmap; do
  f="$PASTA/params_$(echo $n | tr '/' '_').yaml"
  if timeout 15 ros2 param dump /$n > "$f" 2>/dev/null && [ -s "$f" ]; then echo "   ok: $n"; else rm -f "$f"; echo "   não ativo: $n"; fi
done

TOPICOS=(
  /rosout /tf /tf_static /scan_front /scan_raw /odom /imu/data /map /diagnostics
  /plan /plan_smoothed /local_plan /transformed_global_plan /evaluation
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /collision_monitor_state
  /local_costmap/costmap /local_costmap/costmap_updates /local_costmap/published_footprint
  /global_costmap/costmap /global_costmap/costmap_updates
  /goal_pose /clicked_point /behavior_tree_log
  /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
  /follow_path/_action/status /compute_path_to_pose/_action/status
  /spin/_action/status /backup/_action/status
)

trap ':' INT
echo ">>> GRAVANDO em $PASTA/bag  —  aperte Ctrl+C para parar."
ros2 bag record --include-hidden-topics --compression-mode file --compression-format zstd \
  -o "$PASTA/bag" "${TOPICOS[@]}"

echo "fim: $(date '+%F %T')" >> "$PASTA/notas.txt"
cp -r /dev/shm/robo_ros_logs_* "$PASTA/ros_logs/" 2>/dev/null
tar -czf "$PASTA.tar.gz" -C ~/bags "$(basename "$PASTA")"
echo ">>> Teste salvo em: $PASTA"
echo ">>> Arquivo único para enviar: $PASTA.tar.gz ($(du -h "$PASTA.tar.gz" | cut -f1))"
