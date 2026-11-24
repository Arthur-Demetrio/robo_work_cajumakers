#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/robo_recovery:${PYTHONPATH:-}"
colcon --log-base review/colcon_logs build --base-paths src --packages-select robo_navigation robo_recovery --build-base build_candidate --install-base install_candidate
cmake -S tests/mecanum/native_navigation -B build_candidate/native_navigation
cmake --build build_candidate/native_navigation
for config in shim_reference global_unambiguous_candidate; do
  ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF build_candidate/native_navigation/costmap_layer_probe --ros-args --params-file "src/robo_navigation/config/nav2_params_${config}.yaml"
done
python3 tests/mecanum/run_native_navigation.py --horizon --tuning
python3 -m pytest -q src/robo_recovery/test/test_supervised.py tests/mecanum/test_configuration.py tests/mecanum/test_server_contract.py tests/mecanum/test_native_results.py --junitxml=review/antecipacao/pytest.xml
