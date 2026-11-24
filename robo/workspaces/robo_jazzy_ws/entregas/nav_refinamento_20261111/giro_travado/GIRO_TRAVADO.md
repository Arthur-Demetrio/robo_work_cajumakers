# Giro travado no goal: bag 2nav, 12:37:22–12:37:41

**Resumo.** Dois defeitos independentes se somaram:
1. **O goal checker do `controller_server` e o DWB usam `map→odom` de instantes diferentes.** O goal checker converte o fim do caminho com o `map→odom` do momento em que o caminho foi planejado. O DWB usa o mais recente. Com o SLAM corrigindo o `map→odom` o tempo todo (2–10° e 10–30 cm a cada 0,15–0,3 s), o goal checker via o alvo deslocado e **nunca** registrou o XY. Já o `RotateToGoal` do DWB se considerava no goal e girava.
2. **Girando no sentido anti-horário (+), o DWB não consegue frear nem inverter.** A única velocidade de giro parado válida é +0,16. Por isso o robô passou 126° do yaw pedido em vez de oscilar em torno dele.

O defeito 1 é o que impede o goal de terminar. O 2 só transforma isso num giro sem fim.

A menor correção necessária é a do defeito 1: uma única mudança na árvore, sem tocar em nenhum parâmetro do Nav2. **Ela não foi aplicada.**

## 0. Atualização (11/11, 10h): aplicado e refinado

- **GOAL_TF_ATUAL aplicado no `src/`, compilado e instalado. Não está ativo no robô** (o Nav2 em execução não foi reiniciado). Veja `ATIVACAO_GOAL_TF_ATUAL.md`.
- **Refinamento do mecanismo.** O buffer de TF do `Costmap2DROS`, usado pelo `controller_server`, guarda **10 s**, o padrão do tf2. Quando o caminho tem mais de 10 s, a consulta no carimbo antigo dá `ExtrapolationException`, e o `nav_2d_utils::transformPose` cai no TF mais recente. Portanto o defeito só ocorre quando a chegada é **< 10 s** depois do último (re)planejamento, que foi o caso de 12:37:26,58.
- Com o buffer de 10 s, a sonda reproduz **5/5 "Reached the goal!" reais** do bag das 11:48, no mesmo ciclo, tanto na versão atual quanto na corrigida. A versão anterior da sonda, com buffer de 120 s, superestimava o defeito nesse bag.
- O replay de 12:37 continua: a versão atual não encerra; a corrigida encerra às 12:37:29,30.

## 1. O que o código instalado faz (Nav2 1.3.12, pacotes `1.3.12-1noble.20260615`)

Fonte lido na tag `1.3.12` do repositório oficial, mesma versão dos binários instalados.

- **`nav2_controller/plugins/simple_goal_checker.cpp`, `isGoalReached`.**
  - Com `check_xy_` verdadeiro e o robô fora de `xy_goal_tolerance`, devolve `false`.
  - Dentro da tolerância e com `stateful`, faz `check_xy_ = false`; daí em diante só confere o yaw.
  - `reset()` faz `check_xy_ = true`.
- **`nav2_controller/src/controller_server.cpp`.**
  - `setPlannerPath()` faz `end_pose_ = path.poses.back()`, guardando o carimbo da pose, e chama `goal_checkers_[...]->reset()`. Isso vale a cada goal novo e a cada caminho novo entregue por preempção (`updateGlobalPath`).
  - `isGoalReached()` chama, **a cada ciclo**, `nav_2d_utils::transformPose(tf, "odom", end_pose_, …)`, e o retorno é ignorado.
- **`nav_2d_utils/src/tf_help.cpp`.** `tf->transform(in_pose, out_pose, frame)` usa o **carimbo de `in_pose`**: o `map→odom` histórico, interpolado.
- **`dwb_core/src/dwb_local_planner.cpp`, `prepareGlobalPlan`.** O `goal_pose` das críticas é criado com **carimbo zero**, ou seja, o `map→odom` mais recente.
- **`dwb_plugins/src/standard_traj_generator.cpp`.** `startNewIteration(current_velocity, sim_time_)`: a janela dinâmica usa **`sim_time` (0,8 s)** como tempo.
- **`dwb_plugins/include/dwb_plugins/one_d_velocity_iterator.hpp`.** Para subir o valor (com sinal) usa `acc_lim`; para descer usa `decel_lim`.
- **`dwb_plugins/include/dwb_plugins/kinematic_parameters.hpp`, `isValidSpeed`.** Com `|v_xy|` < `min_speed_xy` e `|wz|` < `min_speed_theta`, recusa; recusa também a velocidade zero.
- **`dwb_critics/src/rotate_to_goal.cpp`.** No goal e parado (`rotating_`), qualquer trajetória com translação é ilegal ("Nonrotation command near goal").

**Sobre o `stateful`, que você levantou.** Uma correção de `map→odom` sozinha, de fato, **não** desfaz um XY já registrado. Mas aqui o XY **nunca chegou a ser registrado** no caminho em uso. Há duas razões:
- às 12:37:26,58 o `IsPathValid` do MANTER_CAMINHO falhou, o BT replanejou e o `setPlannerPath()` resetou o checker;
- daí em diante, o alvo convertido com o carimbo 12:37:26,58 ficou a 6,6–11,4 cm do robô, sempre acima de 6 cm.

No caminho anterior (carimbo 12:37:11,76), o alvo do checker estava a 55 cm. Nesses 15 s, o `map→odom` andou 96 cm e 20°.

## 2. Sequência reconstruída do bag

Arquivos: `ciclos_controller.csv` (320 ciclos), `caminhos.csv`, `map_odom.csv`, `eventos_bt_acoes.csv`. Gerados por `scripts/reconstruir_giro_travado.py`.

| Hora | Evento | Robô (odom) | Alvo do goal checker (carimbo do caminho) | Alvo do DWB (TF atual) | wz comandado | Trajetórias válidas |
|---|---|---|---|---|---|---|
| 12:37:11,76 | BT replaneja; caminho novo (carimbo 11,76), checker resetado | — | — | — | — | — |
| 12:37:22,51 | DWB no goal (0,059 m): `RotateToGoal` em janela | (1,175; 0,562) | **(1,284; 1,029), a 0,48 m** | 0,059 m | 0 | 352 |
| 12:37:23–26,5 | Girando −68° → −24° | (1,094; 0,509) | a 0,55 m (nunca dentro) | 0,089 m | +0,16 | **1** |
| 12:37:26,58 | `IsPathValid` FALHA → ComputePathToPose → SmoothPath → caminho novo; `setPlannerPath` → `reset()` | | | | | |
| 12:37:26,7–27,5 | DWB se reposiciona de lado | (1,086; 0,486) | (1,017; 0,464; 16,9°), a 0,07 m | 0,069 m | vy −0,05 | 392 / 244 |
| 12:37:27,4 | SLAM corrige o `map→odom` (≈ −6 cm, −14 cm, +9°) | | o alvo do checker **não muda** (carimbo fixo) | o alvo do DWB muda | | |
| 12:37:28,0 | DWB volta ao giro | (1,130; 0,485) | **0,114 m: XY não registra** | **0,048 m** | +0,16 | **1** |
| 12:37:29,3–30,6 | Yaw passa por 5°: erro do DWB −0,9° | | XY nunca registrado → `false` | **pelo TF atual o goal teria terminado** | +0,16 | 1 |
| 12:37:30,6–38,8 | Continua girando até +134° (126° além) | | | | +0,16 | 1 |
| 12:37:38,8 | "No valid trajectories" | | | | depois −0,16 | 1 |
| 12:37:41,82 | Operador manda o goal seguinte (preempção) | | | | | |

**Por que 1 de 392.**
- A janela de theta é calculada a partir do `wz` da odom (≈ 0,23, limitado a `max_vel_theta` 0,16): de max(−0,16; 0,16 − 0,2·0,8) = **0,00** até 0,16.
- As 7 amostras são 0; 0,027; 0,053; 0,080; 0,107; 0,133; 0,160. Com vx = vy = 0, `isValidSpeed` recusa todas abaixo de `min_speed_theta` 0,15, e o zero também.
- Sobra **+0,16**. As 391 combinações com translação o `RotateToGoal` declara ilegais.
- Girando no sentido horário (−0,16) a janela vai até +0,16 (pelo `acc_lim_theta` 0,4), então os dois sentidos ficam disponíveis. **A trava só existe no giro anti-horário.**

Resposta direta: `min_speed_theta` 0,15 + `max_vel_theta` 0,16 + `decel_lim_theta` −0,2 **impedem frear ou inverter um giro anti-horário**. Mas eles **não impedem concluir o giro**: a +0,16 rad/s (≈ 0,9° por ciclo), a faixa de ±10° do yaw dura ~2,2 s. O goal checker teria encerrado se tivesse o XY registrado.

## 3. Reprodução offline com o código real do Nav2

`bash ferramentas/testes/validar_giro_travado.sh` compila as sondas em `src/robo_navigation/test/sondas_nav2/giro_travado/` e roda `test/test_giro_travado.py`: **4/4 passam** (`pytest_giro_travado.txt`).

| Sonda | Entrada | Resultado |
|---|---|---|
| `goal_checker_replay`: `SimpleGoalChecker` real (pluginlib, `nav2_params.yaml`) + `nav_2d_utils::transformPose` real + `map→odom` do bag entrando no buffer no tempo certo | 320 ciclos do bag | **atual: nunca encerra** (1 ciclo dentro de 6 cm na janela toda). **Carimbo zero: encerra em 12:37:29,30** (144 ciclos dentro) |
| `janela_dwb`: `StandardTrajectoryGenerator` real com os parâmetros do yaml | wz = +0,16 / +0,23 | 392 amostras válidas, giro parado só **[+0,16]** |
| `janela_dwb` | wz = 0 / −0,16 | [−0,16; +0,16] |
| `janela_dwb` com **só** `decel_lim_theta` −0,4 (diagnóstico, não aplicado) | wz = +0,16 | [−0,16; +0,16] |

## 4. Menor correção necessária: `GOAL_TF_ATUAL` (aplicada no src em 11/11 às 10h; NÃO ativa até reiniciar o Nav2)

- **Uma mudança:** o `FollowPath` recebe o caminho através do nó novo `CaminhoSemCarimbo`, que copia o caminho com carimbo zero e mantém frame e poses. Com isso o `end_pose_` do `controller_server` passa a ser convertido com o `map→odom` mais recente, **o mesmo do DWB**.
- **Sem parâmetro do Nav2 alterado**; o `decel_lim_theta` continua −0,2. Biblioteca separada (`robo_caminho_sem_carimbo_bt_node`), independente do Patch 01.
- **Efeito colateral verificado.** O DWB, o `TruncatePathLocal` e o `IsPathValid` não usam o carimbo das poses. O nó é estável: o mesmo caminho com outro carimbo gera a mesma saída (teste), então o FollowPath do BT não vê "caminho novo" à toa. Hoje, cada replanejamento idêntico resetava o goal checker.
- **Validado offline:**
  - gtest do nó (2 testes; pacote com 25/25);
  - parse pelo BT.CPP real com os plugins do Nav2;
  - `bt_navigator` real ativo com a árvore e o yaml modificados (`bt_navigator_isolado_goal_tf.txt`);
  - patch script: aplica, recusa a segunda vez e desfaz byte a byte, numa cópia.
- **Por que não mudar `decel_lim_theta`.** Mudar só ele (−0,2 → −0,4) libera a inversão, mas **não** faz o goal checker encerrar: o DWB ficaria oscilando em torno do yaw até o progress checker ou o operador intervir. Ele fica como segundo teste, separado, só se o físico mostrar ultrapassagem com o `GOAL_TF_ATUAL` ativo.

**Aplicar (só na janela controlada, com o Nav2 parado):**
```bash
python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_goal_tf_atual.py
cd ~/robo_jazzy_ws && colcon build --packages-select robo_bt_plugins robo_navigation && source install/setup.bash
# reiniciar o ~/subir_robo.sh
```
**Desfazer:** `python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_goal_tf_atual.py --desfazer`, recompilar o `robo_navigation` e reiniciar.

## 5. Teste físico supervisionado (área aberta)

**Montagem**
- Área livre de pelo menos 2 × 2 m, longe de paredes (> 0,8 m), sem caixas.
- Operador com parada de emergência na mão. Collision Monitor ativo. Ré bloqueada (padrão).
- Patches 02 e 04 desligados (padrão). O Patch 01 em `registrar` é aceitável: não muda comportamento.
- `giro_final_comum false`.
- Gravar com `~/gravar_teste.sh`: **um bag para a baseline e um bag com o `GOAL_TF_ATUAL`**, mesmo roteiro.

**Roteiro (igual nos dois bags):** 6 goals sem área pelo RViz, a cerca de 2 m do anterior. O yaw do goal deve ficar 90° anti-horário em relação ao rumo de chegada nos goals 1, 3 e 5, e 90° horário nos goals 2, 4 e 6. O giro final no sentido + é o caso que trava. Nenhuma troca de goal antes de terminar ou falhar. Até 60 s por goal.

**Medir em cada goal (scripts em `entregas/nav_refinamento_20261111/scripts/`):**
- `NavigateToPose` SUCCEEDED e "Reached the goal!" no log;
- tempo do início do giro final (DWB parado no goal) até o "Reached";
- erro de yaw final pela TF;
- ultrapassagem: o maior ângulo além do yaw do goal durante o giro;
- saltos do `map→odom` durante o giro final (`map_odom.csv` pelo `reconstruir_giro_travado.py`).

**Critérios de sucesso do `GOAL_TF_ATUAL` (todos):**
1. 6 de 6 goals com SUCCEEDED e "Reached the goal!".
2. Giro final terminado em ≤ (|erro de yaw inicial| / 0,16 rad/s) + 3 s.
3. Erro de yaw final ≤ 10° (tolerância 0,17 rad) e ultrapassagem ≤ 15°.
4. Nenhum giro contínuo além de 30° do yaw do goal.
5. Nenhum toque nem intervenção do operador. Contagem de paradas do CM não maior que a da baseline.

**Leitura da baseline.** Se o giro travar em algum goal, deve coincidir com "XY nunca registrado" e com um salto do `map→odom` (a reconstrução mostra os dois). Se a baseline não travar em nenhum dos 6, o teste não discrimina: repita com goals mais longos (≥ 3 m), onde o `map→odom` corrige mais entre o planejamento e a chegada.

**Abortar o teste:** robô a < 10 cm de algo, giro > 360° ou qualquer comportamento inesperado. Use a parada de emergência e anote a hora.
