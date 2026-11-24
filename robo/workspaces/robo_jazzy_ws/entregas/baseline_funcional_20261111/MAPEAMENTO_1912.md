# Remapeamento 11/11 ~19:10: "demora muito a considerar um ponto"

Só leitura do sistema vivo, mais um patch de config (SLAM_OCC01).
- Filtro do LiDAR (`robo_lidar/scan_filter_node.py`): não tem nada temporal. Cada scan sai na hora, só invertido e sem o corpo (CORPO_REAL_M35).
  Há 1 publicador em `/scan_front`.
- **O "filtro" é o `occupancy_threshold` do slam_toolbox**: 0,2 desde 03/11 (backup `slam_mapping.yaml.bak_occ01`), quando o padrão é 0,1.
  Patch `ferramentas/patches/aplicar_slam_occ01.py` (marca SLAM_OCC01): 0,2 → 0,1, aplicado no src e com build (install conferido).
  Só vale quando o SLAM subir de novo. O limiar muda só o `/map` desenhado, não o casamento de scan.
- O `/map` é republicado a cada 2 s (`map_update_interval` 2,0; medido 0,5 Hz).
- Concorrência de CPU durante o mapeamento: carga 13–19. Três processos de treino RL (`robo_rl_v2/train_gated.py`, ~140% de CPU),
  o alinhador com 78% e o destravar com 56% de CPU com o robô parado. O log do slam_toolbox
  (`~/.ros/log/async_slam_toolbox_node_77824_*.log`) tem 10 avisos "Message Filter dropping message ... queue is full" desde 18:54.
  Hipótese para a perda de orientação: scans descartados, além da derrapagem. Não medido em bag.
- Pendente: investigar a CPU do alinhador e do destravar parados (Python processando cada scan e a vigia).

## 19:38: mapa salvo e rota longa
- Mapa `~/mapas/arena_11-11_1938` (.posegraph/.data e .pgm/.yaml), feito com o filtro do LiDAR corrigido (filtro às 19:22, SLAM às 19:23).
  START/FINISH em `arena_11-11_1938_pontos.yaml` = (0,018; −0,007; 0,043 rad): pose medida com o robô a 1,8 cm do nó 0 do grafo.
  Mesas ainda não marcadas neste mapa.
- Rota longa: quem falha é o **planner do Nav2** ("GridBased plugin failed to plan"). As rotas curtas vão pelo TRAJETO CURTO, sem planner.
- Costmap global ao vivo (escala 0–100): 4.027 letais, **18.593 "inscritos" (99)** e só 4.530 livres. O planner trata 99 como bloqueado.
  O `/map` tem **407 células ocupadas soltas (grupos de ≤ 3)** dentro das áreas livres. Cada uma, inflada pelo raio inscrito, fecha ~50 células.
  (2,14; 4,27) → (0,53; 2,61): BLOQUEADO no costmap, com o destino em custo 99. Sem os pontos soltos: CONECTADO.
  O `rota_no_mapa` do alinhador já ignora grupos com < 3 células, por isso o alinhador "acha" a rota e o Nav2 não.
- Os pontos soltos aumentam com `occupancy_threshold` 0,1 (SLAM_OCC01, de hoje): as paredes aparecem mais rápido, mas o ruído também vira parede.
  Como o /map é refeito a partir dos scans do posegraph, o limiar pode ser ajustado **sem remapear**.
- Próximo passo (com o robô desligado): regenerar o /map do `arena_11-11_1938.posegraph` num domínio isolado com limiar 0,10/0,15/0,20/0,25
  e `min_pass_through` 2/3; medir os pontos soltos, a continuidade das paredes e a conectividade das rotas longas; escolher um valor só.
