#!/usr/bin/env bash
set -eo pipefail
module_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$module_root"
source /opt/ros/jazzy/setup.bash
source "$module_root/install/local_setup.bash"
expected="$module_root/install/robo_virtual_wall"
resolved=$(ros2 pkg prefix robo_virtual_wall)
[[ $(realpath "$resolved") == $(realpath "$expected") ]] || { echo "Overlay incorreto: $resolved" >&2; exit 1; }
export PYTHONPATH="$module_root/src/robo_virtual_wall:${PYTHONPATH:-}"
python3 -m pytest -q "$module_root/src/robo_virtual_wall/test" "$@"
