#!/usr/bin/env bash
# Uso: rodar_slam_replay.sh <dominio> <mapping|localizacao> <bag> <saida_dir> <x> <y> <yaw>
# Sobe o slam_toolbox (launch de producao, use_sim_time) num DOMINIO ISOLADO e toca o bag. Sem hardware.
D=$1; MODO=$2; BAG=$3; OUT=$4; X=$5; Y=$6; TH=$7
WS=$HOME/robo_jazzy_ws; mkdir -p $OUT
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
export ROS_DOMAIN_ID=$D ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
SH=$(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/launch; CFG=$WS/src/robo_navigation/config
if [ "$MODO" = mapping ]; then L=$SH/online_async_launch.py; P=$CFG/slam_mapping.yaml; MT=2; else L=$SH/localization_launch.py; P=$CFG/slam_localizacao.yaml; MT=3; fi
[ -n "$SLAM_PARAMS" ] && P=$SLAM_PARAMS
ros2 launch $L slam_params_file:=$P use_sim_time:=true autostart:=true > $OUT/slam.log 2>&1 &
LPID=$!
sleep 6
python3 -P $WS/entregas/nav_refinamento_20261111/scripts/tocar_bag_slam.py $BAG $OUT/map_odom.csv $HOME/mapas/arena_04-11_1235 $MT $X $Y $TH > $OUT/tocador.log 2>&1
kill -INT $LPID; sleep 3
for p in $(pgrep -f "slam_toolbox_node"); do tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q "ROS_DOMAIN_ID=$D$" && kill -9 $p; done
echo fim > $OUT/fim
