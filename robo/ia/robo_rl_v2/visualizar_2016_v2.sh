#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 CUDA_VISIBLE_DEVICES=""
# Check the new scene and display before closing any existing viewer.
.venv/bin/python -c 'from robo_rl_v2.arena2016v2 import Arena2016V2Env; import tkinter as tk; e=Arena2016V2Env(); e.reset(seed=42); e.close(); r=tk.Tk(); r.withdraw(); r.destroy()'
if command -v wmctrl >/dev/null 2>&1; then
    wmctrl -F -c 'Robo-WORK • AVALIAÇÃO de checkpoints • treino independente' || true
    wmctrl -F -c 'Robo-WORK • Arena 2016 • avaliação do treino transferido' || true
    wmctrl -F -c 'Robo-WORK • Arena 2016 v2 • camadas separadas' || true
fi
exec .venv/bin/python -u -m robo_rl_v2.viewer2016v2 "$@"
