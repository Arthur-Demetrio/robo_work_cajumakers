# Patches do refinamento de 11/11/2026

Baseline: workspace igual ao snapshot de fontes do bag `2nav_integrada_corredor_aberta_20261110_123314` (conferido arquivo a arquivo). Backup datado de todos os arquivos tocados: `backups/baseline_20261111_084923/` (com `SHA256SUMS`, `git_status.txt` e `git_diff_completo.patch`).

**Estado de cada patch.** Os estados são distintos: integrado no source → compilado → instalado → **ativo** → validado fisicamente.

| # | Marca | Arquivos | Padrão | Integrado | Compilado/instalado | Ativo no robô | Teste físico |
|---|---|---|---|---|---|---|---|
| 01 | `CAMINHO_ORIGINAL (11/11)` | pacote novo `src/robo_bt_plugins/`; `behavior_trees/navigate_to_pose_smooth.xml`; `config/nav2_params.yaml` (`bt_navigator.plugin_lib_names`) | **modo `registrar`** (só escreve no log; comportamento igual ao de hoje) | sim | sim (`colcon build`) | **não** (o Nav2 não foi reiniciado) | não |
| 02 | `GIRO_FINAL_COMUM (11/11)` | `scripts/alinhador_passagem.py` (parâmetro `giro_final_comum`) | **desligado** (`false`) | sim | script do src, sem build | não | não |
| 03 | sem mudança de código | `scripts/lateral_vx.py`, `FICHA_CALIBRACAO_LATERAL.md` | não se aplica | ferramentas | não se aplica | não se aplica | não |
| 04 | `ROTA_MAPA_OBSTACULO (11/11)` | `scripts/alinhador_passagem.py` (parâmetro `rota_mapa_obstaculo`) | **`ignorar`** (igual a hoje) | sim | script do src | não | não |
| 05 | `GOALALIGN0 (11/11)` | perfil novo `config/nav2_params_goalalign0.yaml` (o `nav2_params.yaml` não muda) | **não usado** (o launch usa o `nav2_params.yaml`) | sim | sim (instalado no share) | não | não |
| — | `GOAL_TF_ATUAL (11/11)` | `robo_bt_plugins` (nó `CaminhoSemCarimbo`, lib `robo_caminho_sem_carimbo_bt_node`); árvore principal; `plugin_lib_names` | sempre ligado quando o Nav2 sobe com ele | sim (11/11 10h) | sim | **não** (falta reiniciar o Nav2) | não |

Cada patch roda um teste físico por vez. Nunca ligue dois na mesma gravação.

---

## Patch 01 — CAMINHO_ORIGINAL: conferir o caminho original quando a suavização falha

**Problema (medido no bag).** No 2nav, 36 de 54 suavizações (67%) foram reprovadas por colisão. A árvore então mandava o caminho **original** ao FollowPath **sem conferir nada** (ORIGINAL_SEM_GATE).

Onde o smoother acusou a colisão:
- 15 vezes na própria pose do robô;
- 9 perto do goal;
- 12 no meio do caminho.

Meu modelo offline das checagens do Nav2 reproduz 100% dos veredictos do smoother (54/54 e 23/23).

**Por que não usar o `IsPathValid` do Nav2:**
- no Jazzy ele reprova o footprint em custo **INSCRITO** (inflação);
- ele usa o yaw gravado, e o NavFn grava yaw 0 em todas as poses;
- resultado: os 80 caminhos originais do bag saem "inválidos". Foi exatamente por isso que o ORIGINAL_SEM_GATE foi criado.

Com um gate estrito no footprint, 30 de 37 originais seriam reprovados, mas 23 deles se livram com um deslocamento lateral de 2–8 cm. Isso é raspão de grade (5 cm), não bloqueio.

**O que foi feito.** Uma condição de BT nova, `CaminhoOriginalSeguro`, no plugin `robo_caminho_original_seguro_bt_node`.

Ela reprova o original **só se houver bloqueio de fato**:
- o footprint, na orientação da tangente e na última pose com o yaw do goal, cai sobre uma célula LETHAL;
- e nenhum deslocamento lateral de até 10 cm o livra.

Ela não confere:
- as células sob o footprint **atual** do robô (+5 cm);
- os 0,30 m iniciais;
- os 0,60 m finais, a zona do goal, onde o robô gira para o yaw pedido e a tangente não vale.

Uma célula desconhecida não bloqueia, porque o planner tem `allow_unknown: true`. Sem costmap, com costmap velho (> 3 s) ou sem pose, devolve SUCCESS, igual a antes, e avisa no log.

Quando reprova:
- marca `caminho_bloqueado`;
- **não** manda o original;
- a navegação cai na recuperação avaliada (destravar v4), e o `ScriptCondition caminho_bloqueado` garante a recuperação mesmo sem código de erro do smoother.

O replanejamento após a recuperação usa a mesma conferência. Se ela reprovar ali, a pipeline replaneja.

**Replay nos bags (núcleo C++ com o costmap gravado), em modo `aplicar`:**
- 2nav: reprovaria **11 de 38** fallbacks:
  - 7 às 12:35:07–10, na boca do T, onde o DWB de fato dizia "No valid trajectories" e o alinhador assumiu;
  - 3 às 12:37:49–50, caminho para o norte, onde o Nav2 de fato falhou ("Failed to make progress", destravar sem alternativa, Goal failed);
  - 1 às 12:38:53, num caminho que foi trocado 0,5 s depois.
- 11:48: reprovaria 1 de 17.
- Com `folga_fim` 0,25 seriam 18; os 7 extras eram perto de um goal que o robô depois alcançou. Daí o 0,60.

**Por que o padrão é `registrar`:** o replay mostra que as reprovações caem onde o robô de fato travou, mas não prova que recuperar ali é melhor do que deixar o DWB e o CM agirem. Isso só um bag responde.

**Teste A/B (um bag por modo, mesmo cenário):**
```bash
# depois do colcon build e de reiniciar o ~/subir_robo.sh numa janela controlada (modo registrar)
# bag A: modo registrar (o log mostra "CAMINHO_ORIGINAL (registrar): REPROVARIA: ...")
# bag B: modo aplicar, sem reiniciar:
ros2 topic pub --once --qos-durability transient_local /caminho_original/modo std_msgs/msg/String "{data: aplicar}"
# voltar a registrar:
ros2 topic pub --once --qos-durability transient_local /caminho_original/modo std_msgs/msg/String "{data: registrar}"
```
O `ros2 param set` **não** serve aqui: o nó cliente do BT não é atendido por executor e a chamada trava (verificado). Por isso a troca é por tópico.

**O que medir:**
- contagem de `REPROVARIA` / `REPROVADO`;
- se cada reprovação coincidiu com "No valid trajectories" ou com contato;
- tempo até o goal;
- número de destravar.

**Rollback:**
```bash
python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_caminho_original.py --desfazer   # árvore + yaml
cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash
# opcional: touch ~/robo_jazzy_ws/src/robo_bt_plugins/COLCON_IGNORE   (o pacote fica inerte sem o plugin_lib_names)
```
Atenção: com o patch, o `bt_navigator` **exige** o pacote `robo_bt_plugins` compilado e instalado. Sem ele, o Nav2 não ativa. Sempre `colcon build --packages-select robo_bt_plugins robo_navigation` antes de subir.

---

## Patch 02 — GIRO_FINAL_COMUM: o yaw final do goal sem área feito pelo alinhador

**Problema (medido).** Dos giros ≥ 45° do 2nav:

| Origem | Giros | Tempo | Ângulo |
|---|---|---|---|
| Pré-giros do alinhador (VIRAR_CAMINHO) | 6 | 29 s | 456° |
| DWB | 4 | 28 s | 331° |
| TRAJETO CURTO | 1 | — | 92° |

Os 4 giros do DWB foram **todos no goal**, girando para o yaw 5° dos goals manuais.

Às 12:37:22–41 o robô estava a 4–5 cm do goal e o `/evaluation` mostra **1 trajetória válida de 392**: girar a +0,16 rad/s. O motivo:
- `min_speed_theta` 0,15 com `decel_lim_theta` −0,2 a 10 Hz deixa a janela dinâmica em [0,14; 0,16];
- abaixo de 0,15 é inválido;
- então o DWB não consegue frear nem inverter e passou **126° além** do yaw pedido.

O goal checker também não encerrou. No frame odom, às 12:37:29,9 o erro era 0° e 4,8 cm. Há uma explicação que bate com os números: correção do SLAM de 15 cm e 9° às 12:37:27,4, com o goal convertido pelo `map→odom` antigo. Ela está em `RELATORIO.md` e ainda é **hipótese** sobre o código do Nav2.

**O que foi feito.** Parâmetro novo `giro_final_comum` (padrão `false`). Ligado, um goal **sem área** passa pelo mesmo caminho do GIRO_FINAL das mesas:
- o Nav2 leva só até a posição, com o rumo de chegada do caminho e `aceitar_perto`;
- o alinhador gira para o **yaw pedido** pelo `_girar_seguro`, que para e não passa do ponto.

O yaw final **não muda**. As mesas não mudam. O limiar do pré-giro (`virar_caminho_graus` 45) também não.

**Teste A/B:**
```bash
ros2 param set /alinhador_passagem giro_final_comum true    # bag B
ros2 param set /alinhador_passagem giro_final_comum false   # volta
```

**Teste separado (outro bag, sem mudança de código):** pré-giro só acima de 90°:
```bash
ros2 param set /alinhador_passagem virar_caminho_graus 90.0   # volta: 45.0
```

**O que medir:**
- tempo e ângulo dos giros (`scripts/linha_do_tempo_giros.py`);
- erro de yaw final;
- se houve ciscado ou contato.

**Rollback:** `python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_giro_final_comum.py --desfazer`. O desfazer é por troca reversa e funciona em qualquer ordem com o Patch 04. Depois, reiniciar o alinhador.

---

## Patch 03 — Recuo no deslize lateral: diagnóstico e calibração (sem mudança de código)

Veja `TABELA_LATERAL.csv`, `scripts/lateral_vx.py` e `FICHA_CALIBRACAO_LATERAL.md`.

Não mexi no `fundido_k_ff` nem no destravar: a skill exige medir com trena antes. A telemetria de componentes **não** foi adicionada ao alinhador. O modelo do `vx` é determinístico (`vx = −k_ff·vy ± 0,04·des` ou `± 0,03` pela folga) e o script decompõe o `vx` direto do bag.

---

## Patch 04 — ROTA_MAPA_OBSTACULO: medir antes de recusar a rota do mapa

**Problema.** Às 12:33:39 o alinhador trocou o planner (10,5 m) pela rota do mapa (6,1 m). Reproduzi com a própria `rota_no_mapa` sobre o `/map` gravado.

Há 13 células LETHAL do obstacle_layer, livres no mapa, a menos de 0,35 m da rota. Mesmo exigindo mais de 10 cm de distância de qualquer parede do mapa, sobram 8, ao longo da parede norte do corredor, **que o robô atravessou**. O critério ainda não separa "parede vista deslocada" de "objeto". Por isso **não** entrou como regra ativa.

**O que foi feito.** Parâmetro novo `rota_mapa_obstaculo`:
- `ignorar` (padrão, igual a hoje);
- `registrar` (log `ROTA_MAPA_OBSTACULO`);
- `recusar` (com objeto novo na rota do mapa, fica o planner).

Também assina `/global_costmap/obstacle_layer` (só leitura).

**A/B:** `ros2 param set /alinhador_passagem rota_mapa_obstaculo registrar` primeiro, para juntar casos. Só depois um bag com `recusar`, num cenário de caixa no corredor.

**Rollback:** `python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_rota_mapa_obstaculo.py --desfazer`.

---

## Patch 05 — GOALALIGN0: perfil isolado `GoalAlign.scale` 18 → 0

O perfil `early_test`, que já existe, **não serve** para este A/B: muda também `sim_time` (0,8 → 1,6) e usa a árvore reduzida, que falha em vez de recuperar.

O perfil novo `nav2_params_goalalign0.yaml` é o `nav2_params.yaml` com **uma** diferença, conferida pelo script e pelo teste. Ele usa a árvore integrada e o plugin do Patch 01.

**Latência medida** (período do `/cmd_vel_ctrl`):

| Bag | p50 | p95 | Avisos "missed rate" | Trajetórias válidas por ciclo (mediana) |
|---|---|---|---|---|
| 2nav | 100 ms | 106 ms | 0 | 344/352 |
| 11:48 | 99 ms | 128 ms | 6 | — |

No 2nav o controller manteve 10 Hz; os "~160 ms" da análise anterior não se confirmam. No 11:48, 24% dos ciclos passaram de 110 ms.

**Ativar (Nav2 parado, janela controlada):** na linha 108 do `~/subir_robo.sh`, acrescente ao `ros2 launch robo_navigation navigation.launch.py ...`:
```
params_file:=/home/marcos/robo_jazzy_ws/install/robo_navigation/share/robo_navigation/config/nav2_params_goalalign0.yaml
```

**Medir:**
- período do controller e `/evaluation`;
- se a caixa é contornada;
- yaw final;
- giros no goal (`linha_do_tempo_giros.py`).

O GoalAlign 0 não resolve a trava de giro no goal (Patch 02).

**Rollback:** tirar o `params_file:=` do `subir_robo.sh`. Apagar o perfil: `python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_perfil_goalalign0.py --desfazer`.

---

## GOAL_TF_ATUAL — giro travado no goal (aplicado em 11/11, 10h)

Diagnóstico, replay e procedimento: `giro_travado/GIRO_TRAVADO.md` e `giro_travado/ATIVACAO_GOAL_TF_ATUAL.md`.
Rollback: `python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_goal_tf_atual.py --desfazer`, depois `colcon build --packages-select robo_navigation` e reiniciar.
