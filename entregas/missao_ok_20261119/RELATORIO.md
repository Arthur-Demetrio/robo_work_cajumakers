# Missão completa sem ajuda — 19/11/2026

Robô mecanum RoboCup@Work Entry Level (ROS 2 Jazzy). Depois de vários dias de testes, a missão curta
**WS_1 → WS_2 → WS_1 → FINISH** foi concluída sem ajuda humana, com encosto certo nas três mesas.
Este documento registra o resultado, o que foi mudado na sessão de 19/11 (18:50–21:20) e como reproduzir.

## 1. Resultado

Log: `logs/nav2_pre_giro_refaz_211243.log`. Bag (fora do repositório, 4,6 GB): `~/bags/missao_curta_20261119_205056_*`.

| Trecho | Início → fim | Duração | Resultado |
|---|---|---|---|
| → WS_1 | 21:14:06 → 21:14:25 | 18 s | SUCESSO: folga 10,3 cm, ângulo −1,1°, centro +2,8 cm |
| WS_1 → WS_2 | 21:14:30 → 21:15:08 | 38 s | SUCESSO: folga 9,5 cm, ângulo +0,1°, centro +1,1 cm |
| WS_2 → WS_1 | 21:15:18 → 21:15:57 | 39 s | SUCESSO: folga 10,1 cm, ângulo +0,6°, centro +0,6 cm |
| WS_1 → FINISH | 21:16:02 → 21:17:52 | 110 s | SUCESSO (inclui a volta pelo L com saída lateral) |
| **Total** | 21:14:06 → 21:17:52 | **3 min 46 s** | 4/4 trechos sem ajuda |

Observações:
- A missão começou com o robô já perto da WS_1, logo depois da recalibração dessa mesa. Por isso o 1º
  trecho é curto. Ainda falta uma corrida cronometrada saindo do START.
- Nesta corrida, no cruzamento da volta, o alinhamento de saída lateral fez:
  recuo de 5 cm para medir → +14,6° → +7,3° → +1,9° → +2,0°.
  Depois disso o deslize terminou e o robô chegou ao FINISH.
- Os "pegar/soltar" são pausas simuladas (o robô ainda não tem braço).

## 2. Configuração usada

- **SLAM em `localization`**, com mapa fixo `arena_11-11_1938` carregado no START (`match_type` 3).
  Antes, a subida de 19/11 usava `mapping` e às 18:59 o robô perdeu a orientação:
  `map→odom` chegou a −159° de yaw.
- **Subida em 3 terminais:** `ferramentas/subida_unica/abrir_arena.sh` abre:
  1. a base unificada: ponte, LiDAR, odometria, SLAM em localização, parede virtual, mesas e mapa;
  2. o Nav2 legado com fita;
  3. o RViz.
- **Perfil do Nav2** gerado a cada subida por `nav2_legado_com_fita.sh`: o `nav2_params.yaml` de base, mais
  os filtros de fita, mais o patch de horizonte do DWB, mais o **CM_LATERAL65** (novo).

## 3. Mudanças da sessão (cada uma com marca no código e chave para desligar)

| Marca | Arquivo | Problema visto no teste | Mudança | Desligar |
|---|---|---|---|---|
| LOCALIZACAO_FIXA (subida) | `ferramentas/subida_unica/base_arena_localizacao.launch.py`, `slam_localizacao_arena_11-11_1938.yaml` | Em `mapping`, o SLAM se perdeu (yaw `map→odom` −159°) | `localizacao_slam.launch.py` com o perfil gerado por `preparar_localization.py` (só muda o modo, o mapa e a pose inicial); deserialize com `match_type` 3 | usar `base_arena_mapping_0755.launch.py` |
| CHAO_TRASEIRO | `src/robo_lidar/scripts/scan_filter_node.py`, `config/lidar.yaml` | O robô parado fica inclinado e o feixe de trás bate no chão a 1,13–1,35 m (medido em 28 scans) | No setor \|ângulo\| ≥ 130°, retorno ≥ 1,05 m vira NaN (não marca obstáculo nem limpa o costmap) | `floor_filter_enabled: false` |
| CHAO_LADOS | idem | O mesmo chão falso aparece nos lados (relato do operador) | De 60° a 130°, retorno ≥ 1,5 m vira NaN; a frente (±60°) fica sem corte | `floor_side_enabled: false` |
| ALINHAR_CRUZAMENTO_ZERO (v1+v2) | `src/robo_navigation/scripts/alinhador_passagem.py` | Saída lateral na diagonal: o giro parava em +5,6° (pulso de 0,40 s cortado antes do IMU chegar ao ângulo, 3 tentativas, faixa só acima de 3°); em outra corrida, ~11° sem nenhum giro (faixa ≤ 12°, espalhamento ≤ 1,5°) | Até 6 tentativas, corrige de 1° a 16°, espalhamento ≤ 2,5°, pulso `\|ang\|/w_real + 0,25` (teto 1,0 s, quem para o giro é o IMU) e log `ALINHAR_CRUZAMENTO: sem giro: <motivo>` | `alinhar_cruzamento_zero false` |
| RECUO_PARA_MEDIR | idem | Parado no cruzamento com a parede da frente a 7–10 cm (12–15 cm do LiDAR, na zona cega): 0–7 pontos e nenhum alinhamento. A 15 cm, a mesma parede dá 69 pontos | Antes do 1º deslize, sem medida boa e com a frente < 13 cm: recua até ~15 cm (se sobram ≥ 6 cm atrás; no máximo 2 vezes) e mede de novo | `alinhar_cruzamento_zero false` |
| VX_DESLIZE | idem (`_deslizar_fundido`) | No deslize para a direita, `vx = −k_ff·vy + 0,4·des` dava +0,023 com vy −0,08 (direção −74°), o que caía no polígono `diag_dir` do CM (11 cm à frente do para-choque) e o robô parava com a parede a 12 cm. Para a esquerda o vx sai negativo, por isso só a volta falhava | \|vx\| ≤ 0,4·\|vy\| (≤ ~22° da lateral) | — |
| CM_LATERAL65 | `entregas/cm_lateral65_20261119/patch_cm_lateral65.py` (chamado pelo `nav2_legado_com_fita.sh`) | idem | No perfil operacional, o polígono lateral do CM vale de 65° a 135° (era 80°, CM_PROTECAO 11/11); de 45° a 65° continua o diagonal. O `nav2_params.yaml` de base **não** muda (os testes de perfil e o `--desfazer` do CM_PROTECAO dependem dele) | tirar a linha do `nav2_legado_com_fita.sh` |
| MEMORIA_OFF | `src/robo_navigation/scripts/filtro_corpo_scan.py` | A memória do ponto cego (MEMORIA_PERTO) mostrava cópias da face/parede mais perto que a real quando a odometria erra (missão 19: encosto da WS_1 preso por 41 s; 12/11: 4,3 cm com a parede a 14 cm) | Parâmetro `memoria_perto`, **desligado** por padrão. O filtro do corpo (pontos embaixo/dentro do robô) continua | `ros2 param set /filtro_corpo_scan memoria_perto true` |
| PRE_GIRO_REFAZ | `alinhador_passagem.py` (`_navegar_legado`) | Saindo da WS_2, a 2ª tentativa pulou o giro de +46° (trava PRE_GIRO_UNICO, 20 s) e bateu a frente a 0 cm | Se o Nav2 **aborta** (não cancela) para o alvo, a trava é zerada e a próxima tentativa gira para o caminho antes | `pre_giro_refaz_falha false` |

Outros ajustes:
- **WS_1 remarcada** com `marcar_mesa.py`: o encosto falhava com a face medida em 0,64 m.
  - Medida a seco antes de gravar: 0,91 m, estável.
  - Nova face (2,815; 2,176; −3,9°), ~18 cm do cadastro anterior.
- **WS_2 remarcada** pelo operador. O cadastro de 16/11 estava ~40 cm à frente da mesa real, e o scan no
  ponto de aproximação mostrou a face em x ≈ 1,10. Nova face (1,10; 4,16).
- **Robustez do `scan_filter_node`:** usa `getattr` nos atributos novos, porque um teste antigo monta o nó sem `__init__`.

Diferença de código em relação ao início da sessão (backups em `codigo_backups/`):
- `alinhador_passagem.py`: +83 / −9 linhas;
- `filtro_corpo_scan.py`: +12 / −3;
- `scan_filter_node.py`: +38 / −0.

## 4. Testes

- `test/test_alinhar_cruzamento.py`: 27 casos (16 novos), cobrindo:
  - o modo zero (faixa, tentativas, pulso, motivo no log);
  - o RECUO_PARA_MEDIR (poucos pontos, nenhuma medida, sem espaço atrás, desligado, limite de 2 vezes);
  - a direção do deslize dentro do polígono lateral.
- Suítes de cruzamento, curva, lateral, perfis e missão 34/35: 133 aprovados.
- Suíte completa (`ROS_DOMAIN_ID=77`, isolada do robô): 567 aprovados e 3 falhas.
  - `test_patches_desfazer_em_qualquer_ordem_volta_ao_backup` já falhava antes de 19/11.
  - `test_substituicao_goal_ros` e `test_destravar_ros_isolado::test_cancelamento_durante_manobra_para`
    são intermitentes (temporização ROS); passaram ao repetir.
- Filtros de chão: teste de mesa do `ponto_do_chao` (setores e limites) e conferência ao vivo: os retornos
  de trás a ~1,13 m sumiram do `/scan_front`.

## 5. Limitações conhecidas

1. **Hardware:**
   - o USB do Mega (ch341) caiu 3 vezes em 19/11 (10:58, 18:59 e 20:20);
   - a câmera frontal XIFT não aparece, então a parede virtual da fita fica desligada.
2. **Pontos cegos do LiDAR:**
   - os filtros de chão deixam trás e lados com alcance curto (1,05 m e 1,5 m), o que prejudica o mapeamento e a visão lateral;
   - os raios desses setores também não limpam obstáculos antigos no costmap;
   - com a MEMORIA_OFF, um obstáculo que só aparece já colado no para-choque (zona cega de ~11 cm) não é lembrado.
3. **Localização:** na subida, a pose inicial ficou 18–27 cm deslocada do START gravado; o ângulo ficou bom.
4. **Giro no cruzamento:** giros abaixo de ~2° às vezes não movem o robô (6 tentativas a +1,6° sem mudança).
   Candidato: subir o mínimo do modo zero para 2°.
5. **Corrida pendente:** falta uma missão completa saindo do START, cronometrada e com bag próprio.

## 6. Como reproduzir

```bash
# robô no START, Mega e LiDAR conectados
~/robo_jazzy_ws/ferramentas/subida_unica/abrir_arena.sh      # base + Nav2 + RViz
~/gravar_teste.sh <nome> "<descricao>"                        # num terminal separado
cd ~/robo_jazzy_ws/src/robo_navigation/scripts
python3 executor_prova.py ~/robo_jazzy_ws/ferramentas/tarefas/curta_ws1_ws2_ws1.yaml
```

Para o `abrir_arena.sh` funcionar, as cópias desta pasta precisam estar nos caminhos originais:
- `~/mapas/` ← `mapas/`; o `service_areas_ativo.yaml` vai como `service_areas.yaml`, e `arena_11-11_1938_pontos.yaml` também como `pontos.yaml`;
- `~/logs/reabertura_20261116_100105/` ← `subida/reabertura_20261116/`;
- `~/logs/parede_virtual_20261116/parede_virtual.rviz`;
- `~/logs/inicializacao_20261119_075533_iAQf/base.launch.py` ← `subida/base_arena_mapping_0755.launch.py`, só para o modo mapping.

Os caminhos absolutos estão em `abrir_arena.sh`, `rviz.sh`, `parede.sh` e `nav2_legado_com_fita.sh`.

## 7. Conteúdo desta pasta

| Pasta | Conteúdo |
|---|---|
| `mapas/` | Mapa `arena_11-11_1938` (`.posegraph`, `.data`, `.pgm`, `.yaml`), START/FINISH (`_pontos.yaml`) e mesas recalibradas (`_service_areas.yaml` = `service_areas_ativo.yaml`) |
| `subida/` | `abrir_arena.sh`, `base_arena_localizacao.launch.py`, perfil de localização, `subir_tudo.sh` + `base.launch.py` (mapa novo), launch antigo em mapping, scripts da reabertura de 16/11 e o perfil do Nav2 com os patches |
| `tarefas/` | `curta_ws1_ws2_ws1.yaml` |
| `logs/` | Logs da base e do Nav2 de 19/11, incluindo a corrida de sucesso (`nav2_pre_giro_refaz_211243.log`) e as falhas que motivaram cada mudança |
| `codigo_backups/` | Versões de `alinhador_passagem.py`, `filtro_corpo_scan.py` e `scan_filter_node.py` do início da sessão |
| `MANIFESTO_sha256.txt` | Hashes do código alterado e dos arquivos desta pasta |

O código alterado está nos caminhos normais do workspace (`src/robo_lidar/...`, `src/robo_navigation/...`,
`ferramentas/subida_unica/`, `ferramentas/tarefas/`, `entregas/cm_lateral65_20261119/`).
