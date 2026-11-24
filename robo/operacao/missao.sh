#!/usr/bin/env bash
# MISSAO v1 (05/11/2026): grava o bag e roda a missao MISSAO2 num terminal so.
# Uso:  ~/missao.sh missao17 "o que mudou neste teste"  [arquivo_de_tarefa.yaml]
# No fim (ou no Ctrl+C) para a gravacao do jeito normal (como o Ctrl+C no gravar_teste.sh).
set +u
source /opt/ros/jazzy/setup.bash; source ~/robo_jazzy_ws/install/setup.bash
NOME=${1:?nome do teste}; DESC=${2:-}
E=$(grep -l "PLANO_V2" ~/Downloads/executor_prova*.py ~/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova*.py 2>/dev/null | head -1)
T=${3:-$(grep -l "MISSAO2" ~/Downloads/tarefa_missao2*.yaml 2>/dev/null | head -1)}
[ -n "$E" ] && [ -n "$T" ] || { echo "Nao achei o executor ($E) ou a tarefa ($T)."; exit 1; }
setsid ~/gravar_teste.sh "$NOME" "$DESC" &
G=$!
parar_gravacao() { trap - INT TERM EXIT; echo ">>> Parando a gravacao..."; kill -INT -- -$G 2>/dev/null; wait $G 2>/dev/null; echo ">>> Gravacao parada."; }
trap parar_gravacao INT TERM EXIT
sleep 6   # a gravacao comeca a assinar os topicos
python3 "$E" "$T"
