#!/usr/bin/env bash
# Build only this snapshot. Never writes the operational workspace/install.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
colcon --log-base review/install_logs build --base-paths src --packages-select robo_navigation robo_recovery --build-base build_candidate --install-base install_candidate
printf '%s\n' "Overlay candidato: $PWD/install_candidate/setup.bash" "A navegação ativa não foi alterada."
