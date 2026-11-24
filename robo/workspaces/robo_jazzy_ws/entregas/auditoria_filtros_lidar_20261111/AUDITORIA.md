# Filtros do LiDAR (robo_lidar × filtro_corpo_scan) — 11/11/2026 ~19:15

Amostra ao vivo, só leitura (`scripts/amostra_scan.py`, 30 s, robô parado): 185 `/scan_raw`, 191 `/scan_front`, 190 `/scan_nav`, 180 raios a 2°.

## Cadeia
`/scan_raw` → robo_lidar `scan_filter` (inverte os ângulos e tira o corpo) → `/scan_front` → SLAM, alinhador.
`/scan_front` → `filtro_corpo_scan` (tira o corpo e faz a MEMORIA_PERTO) → `/scan_nav` → Collision Monitor e costmaps.

## Achado
- Inversão raw → front conferida: 9.160 de 9.160 raios iguais.
- **O `/scan_front` tinha autorretornos DENTRO do corpo, sempre nos mesmos raios.** Eles estavam em 100% dos scans, com desvio ≤ 7 mm:
  −156 a −140°, −132 a −114°, +104 a +114°, +124 a +142° e **+162 a +166°**. São 6.274 retornos em 185 scans (22% dos válidos).
  Em base_link: x +0,13 a +0,26 e |y| < 0,29, e na traseira esquerda x −0,12 a −0,16, y +0,12 a +0,15.
- O `filtro_corpo_scan` tirava esses pontos do `/scan_nav`, então o CM e os costmaps estavam limpos. **O SLAM e o alinhador recebiam os pontos.**
- Causa:
  - o `lidar.yaml` antigo (git HEAD) bloqueava `-162:-146,-140:-110,100:138,142:160`. O +162..166 já escapava;
  - em 11/11 às 12:36 (CORPO_REAL_M35), os setores foram zerados e trocados pela máscara do corpo no nó;
  - no `install/` operacional (`--symlink-install`), o yaml e o launch são **links** e mudaram na hora, mas o nó é instalado com `RENAME`
    e por isso é uma **cópia**. Ficou a de 01/10, sem máscara;
  - resultado: desde 12:36 o operacional não filtrava nada no `/scan_front`.
- Efeito provável (hipótese, coerente com o que você viu): pontos fixos no robô entram no casamento de scan e puxam a pose para "parado"
  e para a orientação errada, além de sujar o mapa ao longo do trajeto.
  Missões e mapeamentos depois de 12:36 rodaram assim, incluindo a missao35 e o teste das 18:44.

## Correção
- `colcon build --symlink-install --packages-select robo_lidar`: o nó instalado agora é igual ao src (CORPO_REAL_M35).
  Backup do nó antigo em `backups/lidar_corpo_20261111_191954/`.
- Conferência offline com os scans crus gravados: o nó do src tira os 6.274 retornos de dentro do corpo e **nenhum** fora dele.
  É o mesmo retângulo do `filtro_corpo_scan` (x −0,215..0,375, |y| < 0,325), então os dois filtros passam a coincidir.
- Teste de proteção `src/robo_navigation/test/test_install_coincide.py`: falha se a cópia instalada do `scan_filter_node` ou do
  `command_imu_odometry_node` (os dois com RENAME) ficar diferente do src, ou se o yaml ficar sem setores e sem máscara.
- **Só vale quando o filtro do LiDAR subir de novo** (`~/subir_robo.sh ...`). O processo que está rodando continua com o nó antigo.
- Rollback: `cp backups/lidar_corpo_20261111_191954/scan_filter_node.install_antigo install/robo_lidar/lib/robo_lidar/scan_filter_node`
  e voltar os setores no `lidar.yaml`. Não recomendado.

Observação: na amostra, a `/odom` saltou uma vez de (−5,48; 0,61) para (0; 0) aos 8,7 s, um reset (subida/localização reiniciada), não deriva.
