#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/robo_recovery:${PYTHONPATH:-}"
colcon --log-base review/colcon_logs build --base-paths src --packages-select robo_navigation robo_recovery --build-base build_candidate --install-base install_candidate
python3 -m pytest -q src/robo_recovery/test/test_supervised.py tests/mecanum/test_configuration.py tests/mecanum/test_server_contract.py --junitxml=review/pytest.xml
cmake -S tests/mecanum/dwb_probe -B build_candidate/dwb_probe
cmake --build build_candidate/dwb_probe
build_candidate/dwb_probe/bt_probe src/robo_navigation/behavior_trees/navigate_to_pose_ab_candidate.xml src/robo_navigation/behavior_trees/navigate_to_pose_mecanum_candidate.xml
for profile in dwb_candidate shim_reference; do
  ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF build_candidate/dwb_probe/dwb_probe --ros-args --params-file "src/robo_navigation/config/nav2_params_${profile}.yaml"
done
