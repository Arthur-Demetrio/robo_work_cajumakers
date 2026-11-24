#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/robo_recovery:${PYTHONPATH:-}"
bash tests/mecanum/run_anticipation_checks.sh
cmake -S tests/mecanum/dwb_probe -B build_candidate/dwb_probe
cmake --build build_candidate/dwb_probe
build_candidate/dwb_probe/bt_probe src/robo_navigation/behavior_trees/navigate_to_pose_candidate.xml
build_candidate/dwb_probe/bt_probe > review/plugin_ports.xml
ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF build_candidate/dwb_probe/dwb_probe --ros-args --params-file src/robo_navigation/config/nav2_params.yaml > review/main_dwb_load.log 2>&1
ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF build_candidate/native_navigation/native_navigation --ros-args --params-file src/robo_navigation/config/nav2_params.yaml -p experiment.scenario:=lateral -p experiment.output:="$PWD/review/main_lateral.json" > review/main_lateral.log 2>&1
cmake -S tests/mecanum/truncate_probe -B build_candidate/truncate_probe
cmake --build build_candidate/truncate_probe
build_candidate/truncate_probe/truncate_probe > review/truncate_result.log
cmake -S tests/mecanum/native_navigation -B build_candidate/scan_replan
cmake --build build_candidate/scan_replan --target scan_replan_probe
ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF build_candidate/scan_replan/scan_replan_probe --ros-args --params-file src/robo_navigation/config/nav2_params.yaml > review/scan_replan_result.log 2>&1
python3 -m pytest -q tests/mecanum/test_delivery.py tests/mecanum/test_configuration.py tests/mecanum/test_server_contract.py tests/mecanum/test_native_results.py src/robo_recovery/test/test_supervised.py --junitxml=review/pytest_delivery.xml
python3 scripts/report.py
