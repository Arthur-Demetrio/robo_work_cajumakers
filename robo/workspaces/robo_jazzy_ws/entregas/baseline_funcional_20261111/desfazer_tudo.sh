#!/usr/bin/env bash
# Desfaz a rodada BASELINE_FUNCIONAL (11/11) na ordem inversa da aplicacao. Nao mexe em processos vivos.
# Depois: colcon build --packages-select robo_navigation (por causa do launch).
set -e
P=~/robo_jazzy_ws/ferramentas/patches
for s in aplicar_slam_occ01 aplicar_passagem_bloqueada aplicar_envio_goal_seguro aplicar_rota_segura_launch aplicar_ciscado_progresso aplicar_pre_giro_unico \
         aplicar_controlador_orfao_alinhador aplicar_controlador_orfao_destravar aplicar_controlador_orfao_recuperacao \
         aplicar_baseline_launch aplicar_baseline_subida aplicar_baseline_legado aplicar_face_lidar_m24; do
  echo ">>> $s --desfazer"; python3 "$P/$s.py" --desfazer
done
echo "Testes novos (opcional apagar): test_baseline_legado.py test_controlador_orfao.py test_controlador_orfao_ros.py test_melhorias_legado.py test_substituicao_goal_ros.py test_passagem_bloqueada.py"
echo "Backup completo anterior: ~/robo_jazzy_ws/backups/baseline_funcional_20261111_161212/"
