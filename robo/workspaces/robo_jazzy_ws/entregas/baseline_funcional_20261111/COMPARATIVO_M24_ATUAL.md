# Comparativo missao24 (git 7201367) × estado atual — 11/11/2026

Fonte: `git diff 7201367` e as marcas `NOME (dd/mm)` no código. Só leitura; nada foi ativado.
O `install/` operacional está igual ao `src/` para a árvore e o `nav2_params.yaml` (conferido com `cmp`).

## Alinhador (`alinhador_passagem.py`: +1157/−103 linhas)
| Marca nova | Origem | Situação na baseline |
|---|---|---|
| FREIO_CRUZAMENTO, PERDA_PARA, FACE_FRAGMENTADA, CM_RIGIDO | 09/11, rodaram nas missões 25–32 | mantidas |
| GIRO_FINAL_COMUM, ROTA_MAPA_OBSTACULO, REINSISTIR, CURVA_INSISTE | 11/11 | REINSISTIR desligado pelo launch (`reinsistir_goal_final:=0`); ROTA_MAPA_OBSTACULO em `recusar` (rota segura) |
| BASELINE_LEGADO | 11/11 (esta rodada) | `encosto_referencia_mapa` false = comportamento da missao24 |
| CONTROLADOR_ORFAO_ALINHADOR, PRE_GIRO_UNICO, CISCADO_PROGRESSO | 11/11 (esta rodada) | melhorias, cada uma com parâmetro |
| ENVIO_GOAL_SEGURO | 11/11 (esta rodada, depois do relatório) | correção de substituição de goal, ver abaixo |
| Código do supervisor/nav2_principal | 11/11 | presente, inativo (`nav2_principal` false no launch) |

`face_ws.py`: FACE_ANG20, FALHA_SCAN, MOTIVO_FACE, FACE_FRAGMENTADA (09/11) + FACE_LIDAR_M24 (esta rodada: a face é escolhida só pelo LiDAR).

## Interfaces de goal
Na missao24 e agora (iguais): `/goal_pose_alinhado` (PoseStamped), `/ir_para_area` → resultado, NavigateToPose,
ComputePathToPose, `destravar` (DriveOnHeading).
Só no atual:
- usados no legado: `/cmd_vel_ctrl` (sub., CONTROLADOR_ORFAO), `/follow_path/_action/cancel_goal` (cliente),
  `/hardware/sensor_timing`, `get_parameters` do CM e da ponte, `/global_costmap/obstacle_layer`;
- curva em T (arbitro_t): `/cmd_vel_nav_t`, `/alinhador/controle_t`, `/alinhador/cancelar_t`, `/alinhador/estado_t`, `/alinhador/conflito_t`;
- só no nav2_principal (inativos no legado): `/navegacao/especial`, `/navegacao/heartbeat`, `/navegacao/cancelar`.

## destravar
missao24: `destravar_node.py` v3 (bloqueio virtual, CONTORNAR, giro/andar/ré/lado). `recuperacao_avancada.py` **não existia**.
Agora: v4 (NAV_INTEGRADA 10/11) + CONTROLADOR_ORFAO_NO (esta rodada). Já faz o que o item 8 pede:
- escolhe a manobra pelo footprint contra o scan (`avaliar`);
- confirma o resultado por ICP do LiDAR/IMU, e não pela odometria (`verificar_translacao`);
- `Historico` bloqueia repetir no mesmo lugar uma manobra que falhou ou não resolveu, e marca como "sem efeito" quando o
  robô volta ao mesmo bloqueio;
- falha explícita depois de 3 retornos ou 10 manobras por goal.
Ré bloqueada (`permitir_re=false`). Não mudei a lógica de decisão nesta rodada.

## Árvore do Nav2 (`navigate_to_pose_smooth.xml`: +85/−36)
missao24: ComputePath → SmoothPath → FollowPath, recuperação RoundRobin (ClearCostmap / Spin / DriveOnHeading).
Agora: NAV_INTEGRADA (caminho conferido a 2 Hz, recuperação = destravar e replaneja), GOAL_TF_ATUAL, CAMINHO_ORIGINAL,
ORIGINAL_SEM_GATE e **SEM_SUAVIZADOR**. `CaminhoOriginalSeguro` está em `modo="registrar"` (não veta).

## nav2_params.yaml (+82/−41)
NAV_INTEGRADA (DWB direto, global consolidado), GOAL_TF_ATUAL, CAMINHO_ORIGINAL, CM_RIGIDO e **CM_PROTECAO**.
Os parâmetros do DWB não foram mexidos nesta rodada. `min_vel_x` é 0.

## ⚠ Atenção: duas mudanças da M34 no perfil legado ainda não rodaram no robô
O CLAUDE.md registra que SEM_SUAVIZADOR e CM_PROTECAO (sessão M34, 11/11 de manhã) "não foram ativados no Nav2".
As missões 33–35 rodaram em nav2_principal, cujo perfil não tem esses dois patches. Como o install legado já os tem,
**o próximo teste no legado será o primeiro com eles**, junto com a baseline. Isso fere o "uma mudança por vez".
- SEM_SUAVIZADOR: o FollowPath segue o caminho cru do NavFn, sem o simple_smoother.
- CM_PROTECAO: polígonos `forward_veloz`/`diag_esq`/`diag_dir`; o `forward_rapido` vai até 0,12 m/s (era 0,35).
Recomendação: fazer o 1º bag do legado com eles (o CM_PROTECAO é proteção). Se o trajeto ficar diferente da missao24,
o primeiro suspeito é o SEM_SUAVIZADOR, que tem `--desfazer` em `ferramentas/patches/`. Não desfiz nada sem a sua decisão.
