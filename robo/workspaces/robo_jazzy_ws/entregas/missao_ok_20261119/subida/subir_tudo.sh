#!/usr/bin/env bash
# SUBIR_TUDO (19/11/2026): o mesmo conjunto aberto em 19/11 07:55, mas NUM TERMINAL SO:
#   base.launch.py (ponte, LiDAR, localizacao, SLAM DO ZERO - mapa novo, parede virtual)
#   -> Nav2 legado com fita (nav2_legado_com_fita.sh: alinhador_passagem, nav2_principal=false, giro de abertura)
#   -> RViz.
# Logs completos em ~/logs/subida_<data>/; aqui so as linhas importantes. Ctrl+C para tudo (ordem inversa).
# MAPA_NOVO (19/11): nao carrega a arena; o mapa comeca onde o robo estiver.
set +u
D=$(dirname "$(readlink -f "$0")")
LOG=~/logs/subida_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1
PIDS=()
parar_tudo() {
  trap - INT TERM EXIT
  echo; echo ">>> Parando tudo..."
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -INT -- -"${PIDS[$i]}" 2>/dev/null; sleep 2; done
  sleep 3
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -TERM -- -"${PIDS[$i]}" 2>/dev/null; done
  sleep 1
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -KILL -- -"${PIDS[$i]}" 2>/dev/null; done
  echo ">>> Parado. Logs em $LOG"
}
trap parar_tudo INT TERM EXIT
sobe() { local nome=$1; shift; echo ">>> $nome: $*"; setsid "$@" > "$LOG/$nome.log" 2>&1 & PIDS+=($!); }

if pgrep -f "[s]lam_toolbox|[h]ardware_bridge_node|[b]t_navigator" >/dev/null; then
  echo "!!! Ja tem ponte/SLAM/Nav2 rodando. Feche antes:"; pgrep -af "[s]lam_toolbox|[h]ardware_bridge_node|[b]t_navigator" | cut -c1-120
  trap - EXIT; exit 1
fi

sobe base ros2 launch "$D/base.launch.py"
sobe nav2 bash ~/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.sh
echo ">>> Esperando o bt_navigator ficar ativo (o nav2 espera a base/TF sozinho)..."
fim=$((SECONDS + 240)); ok=0
while (( SECONDS < fim )); do
  ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active" && { ok=1; break; }
  sleep 3
done
(( ok )) || { echo "!!! Nav2 nao ficou ativo em 240 s. Veja $LOG/nav2.log"; tail -15 "$LOG/nav2.log"; exit 1; }
sobe rviz bash ~/logs/reabertura_20261116_100105/rviz.sh
sleep 5
echo; echo "---- Conferencia ----"
for n in alinhador_passagem destravar suavizador_nav filtro_corpo_scan collision_monitor; do
  ros2 node list 2>/dev/null | grep -qx "/$n" && echo "    ok: /$n" || echo "    !!! falta /$n"
done
echo "    nav2_principal: $(grep -o 'nav2_principal:=[a-z]*' "$LOG/nav2.log" | head -1; grep -m1 'Alinhador operacional' "$LOG/nav2.log")"
echo "    ganhos laterais da ponte: $(for r in fl fr rl rr; do ros2 param get /robo_hardware lateral_wheel_gain_$r 2>/dev/null | awk '{print $NF}'; done | tr '\n' ' ')"
echo
echo "================ ROBO PRONTO ================"
echo " Confira no RViz se o scan bate com o mapa. Ctrl+C aqui para tudo. Logs: $LOG"
echo "============================================="
tail -n0 -F "$LOG/nav2.log" "$LOG/base.log" 2>/dev/null | grep --line-buffered -E "alinhador|executor|destravar|DESTRAVAR|filtro_corpo|Giro de abertura|ERROR|FATAL|process has died" \
  | grep --line-buffered -v "timestamps differ"
