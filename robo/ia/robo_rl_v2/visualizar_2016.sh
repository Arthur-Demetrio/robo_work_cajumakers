#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export CUDA_VISIBLE_DEVICES=""
# Verify graphical access before asking the old viewer window to close.
.venv/bin/python -c 'import tkinter as tk; root=tk.Tk(); root.withdraw(); root.destroy()'
if command -v wmctrl >/dev/null 2>&1; then
    wmctrl -F -c 'Robo-WORK • AVALIAÇÃO de checkpoints • treino independente' || true
fi
exec .venv/bin/python -u -m robo_rl_v2.viewer2016 "$@"
