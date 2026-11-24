#!/usr/bin/env bash
# Uso: replay_slam.sh <params.yaml> <saida.csv>   (dominio ROS isolado; nao toca no robo)
set +u
P=$1; OUT=$2; D=$(cd "$(dirname "$0")/.." && pwd)
BAG=$D/../costmap_sem_conexao_20261111/ensaio_goal_superior
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 $D/scripts/relay_odom_tf.py "$OUT" > "$OUT.relay.log" 2>&1 & R=$!
/opt/ros/jazzy/lib/slam_toolbox/localization_slam_toolbox_node --ros-args --params-file "$P" -p use_sim_time:=true > "$OUT.slam.log" 2>&1 & S=$!
sleep 6
ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1; ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1
sleep 3
ros2 bag play "$BAG" --clock 50 --topics /scan_front /tf /tf_static --remap /tf:=/tf_bag \
  --qos-profile-overrides-path $D/replay_slam/qos.yaml --playback-duration 75 > "$OUT.play.log" 2>&1
sleep 2
kill -INT $S $R 2>/dev/null; sleep 2; kill -9 $S $R 2>/dev/null; wait 2>/dev/null
echo "linhas $(wc -l < "$OUT")"
