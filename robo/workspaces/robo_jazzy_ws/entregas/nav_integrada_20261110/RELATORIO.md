# NAV_INTEGRADA — navegação antecipativa + recuperação avançada (10/11/2026)

Integrado em `src/robo_navigation/` e instalado (`install/` aponta para os mesmos arquivos). **Nada rodou no
robô.** O Nav2 de teste que estava ativo (`navigation_test.launch.py`, perfil `early_test`) não foi tocado: os
três arquivos dele têm os mesmos SHA-256 do backup e o processo continua com o que carregou.

Backup completo antes de qualquer mudança: `backups/integracao_nav_20261110_102801/` (git status, HEAD, diff
binário, tar de `src/robo_navigation`, `src/robo_recovery`, `src/robo_progressive_spin` e dos `install/`
correspondentes, `sha256.txt`). Cada arquivo alterado tem também `<arquivo>.bak_navintegrada_20261110_105242`.

## 1. O que existia e o que mudou

| Item | Antes | Agora |
|---|---|---|
| Árvore ativa no `navigation.launch.py` | `navigate_to_pose_smooth.xml` (RECUPERACAO_RAPIDA/ARVORE_V3/MANTER_CAMINHO, 1 Hz, RoundRobin) | `navigate_to_pose_smooth.xml` integrada; a anterior ficou em `navigate_to_pose_smooth_anterior_20261110.xml` |
| Controle | RotationShim + DWB | DWB direto (`dwb_core::DWBLocalPlanner`) |
| Recuperação | RoundRobin: destravar 15 → limpar → destravar 25 → smart_spin, às cegas | `destravar` v4 decide uma manobra por diagnóstico, confirma e mantém histórico |
| Candidata do Codex | `supervised_escape` isolado; exigia pose em `recovery/observed_pose` que **nenhum nó publica** e assumia `cmd_vel_ctrl → cmd_vel_nav` (lugar do `suavizador_nav`) | **não usado**; a lógica geométrica dele (volume varrido observado, histórico, re → reavaliar) foi reescrita dentro do `destravar` existente |
| Árvore "reduzida" (`navigate_to_pose_early_test.xml`) | sem recuperação: qualquer falha do DWB ou da suavização encerrava a navegação | mantida intacta, só para o perfil de teste |

## 2. Árvore principal

`behavior_trees/navigate_to_pose_smooth.xml`:

- **Planejamento a 2 Hz** com MANTER_CAMINHO: replaneja só com goal novo, caminho com mais de 15 s ou caminho
  inválido. A validade é conferida no **trecho restante** (`TruncatePathLocal` com 0 m para trás), ou seja, num
  obstáculo atrás do robô não replaneja.
- **Suavização com checagem de colisão**; se falhar, usa o caminho original do planner **se** `IsPathValid` o
  aprovar (`SetBlackboard value="{path}"`, provado com BT.CPP 4.9 real). Se nenhum dos dois servir, não segue.
- Planner sem caminho: limpa o costmap global uma vez (como antes).
- **Recuperação**: só com erro recuperável do controle/planner/suavização → `destravar` (0,40 m, 30 s) →
  sempre recalcula e confere o caminho. Goal novo interrompe a recuperação. Até 6 rodadas; o próprio
  `destravar` falha antes se o bloqueio se repete.
- Compatibilidade: XML aceito pelos plugins reais do Nav2 Jazzy 1.3.12 (`VerifyXML` + registro). Não há
  plugin de BT próprio; os servidores próprios (`destravar`, `smart_spin`) usam `DriveOnHeading`/`Spin`.

## 3. DWB direto

`config/nav2_params.yaml`: só `plugin` mudou e saíram as 9 chaves exclusivas do Shim. Velocidades,
acelerações, `sim_time` 0,8, todos os críticos e pesos (GoalAlign 18 inclusive), goal checker (xy 0,06,
yaw 0,17), progress checker, footprint, velocity smoother e Collision Monitor: iguais (teste compara
carregado = `nav2_params_dwb_test.yaml`). O DWB real carregado com este YAML gera amostras laterais puras
para os dois lados, diagonais, e nenhuma de ré (`min_vel_x 0` mantido).

**Perde-se:** o giro rápido do Shim (0,40 rad/s antes de seguir). Agora o giro do DWB é limitado a
`max_vel_theta` 0,16. A orientação final continua obrigatória (RotateToGoal + goal checker 0,17 rad).

## 4. Global costmap

Tirado o **1º** bloco `obstacle_layer` duplicado (5,0/5,5 m). O efetivo (o último: `/scan_nav`, marcação e
limpeza, 3,5/4,0 m, `inf_is_valid`) não muda; o patch recusa gravar se o dict carregado mudar. Prova em
memória com as camadas e o Navfn reais: um scan novo faz o caminho mudar de passagem
(`after_max_y > before_max_y`, `unsafe_plan_cells=0`).

## 5. Recuperação avançada (`scripts/destravar_node.py` v4 + `scripts/recuperacao_avancada.py`)

O servidor é o mesmo (`nav2_msgs/DriveOnHeading` "destravar"), então a árvore e o ANTI_CISCADO do alinhador
continuam chamando igual. A decisão está em `recuperacao_avancada.py` (sem ROS, o mesmo código roda nos
testes de simulação).

**A — diagnóstico** (vai para o log `DESTRAVAR: DIAGNOSTICO:` e para `/destravar/estado`):
`caminho_invalido` (IsPathValid do /plan), `sem_caminho_global`, `dwb_sem_trajetoria` (controlador parou de
comandar com o planner ok), `cm_bloqueou` (STOP nos últimos 3 s), `sem_progresso_fisico` (comandou ≥ 2 s e o
**ICP do LiDAR** mostra < 3 cm; a odom não conta), `oscilacao` (≥ 4 inversões em 4 s), `quina_esq/dir`,
`frente_barrada`.

**B — contorno normal ainda existe?** Se o planner tem caminho, o corpo inteiro passa 15 cm na direção dele,
a área está observada, o CM não barraria e a causa não é oscilação/sem progresso/CM → **não mexe** e devolve
ao Nav2 (`retomar`). Só uma vez por lugar.

**C — lateral:** CONTORNAR do v3 (objeto pequeno), depois lateral 10–40 cm. Exige corpo inteiro **com a
deriva para a frente medida** (0,4 m/m, parâmetro), área ≥ 50% observada, polígono do CM livre e
**continuação**: da pose final o corpo precisa andar 15 cm na direção do caminho (só direções que o DWB faz,
±95°). O lado é escolhido pela continuação, não pelo raio mais longo.

**D — ré:** só com `permitir_re=true` **e**: área varrida sem ponto e ≥ 90% observada, polígono traseiro do CM
livre, ICP capaz de confirmar o deslocamento ali (pré-teste de ambiguidade), CM ativo (lifecycle), comando
exclusivo. O CM parando interrompe na hora. Cada recusa vai para o log com a causa.

**E — depois da ré:** para, tira foto nova parada, confirma pelo ICP, replaneja e **decide de novo** (nunca
duas rés; nunca ré seguida de lateral/giro sem nova avaliação). Ré não confirmada não é sucesso.

**F — giro:** só se o caminho está a > 45°, nada de lado resolve, o volume girado ±40° está livre e observado
e o CM não barraria. Executado pelo `smart_spin` (micro-giros com arco conferido no costmap e yaw por TF) e
confirmado pela IMU. O giro frontal experimental continua desligado (`SweptTurnStop.enabled false`).

**G — histórico** por goal: o que falhou, não se confirmou ou "deu certo" e o robô voltou ao mesmo lugar
(15 cm) não se repete. 3 retornos ao mesmo bloqueio ou 10 manobras no goal → falha explícita. Sem alternativa
→ `FALHA: sem alternativa segura (...)` com a recusa de cada ação.

**Cadeia de comando** (medida com `ros2 topic info -v`, só leitura): `controller_server → /cmd_vel_ctrl →
suavizador_nav → /cmd_vel_nav`; `destravar`, `alinhador`, `behavior_server` (smart_spin/spin) e
`mecanum_escape_node` também publicam em `/cmd_vel_nav` → `velocity_smoother → /cmd_vel_smoothed →
collision_monitor → /cmd_vel → robo_hardware`. Nenhum servidor contorna o CM. O `destravar` v4 marca os
próprios comandos e **para** se outro nó publicar não nulo em `/cmd_vel_nav` ou se o controlador comandar;
para com cancelamento, timeout, scan/odom com mais de 0,5 s ou CM inativo. **A `/odom` é derivada do
comando** (`command_imu_odometry` assina `/cmd_vel`): só serve como critério de parada (de lado superestima,
então o robô anda menos, nunca mais), nunca como prova.

## 6. Recuperações autorizadas e bloqueadas (padrão)

| Ação | Estado | Liga/desliga |
|---|---|---|
| Devolver ao Nav2 sem mexer (contorno normal) | autorizada | sempre |
| CONTORNAR (v3) com checagem de deriva e CM | autorizada | `permitir_lateral` |
| Lateral com continuação | autorizada | `permitir_lateral` |
| Andar na direção do caminho (com continuação) | autorizada | — |
| Giro pelo smart_spin | autorizada | `permitir_giro` |
| Limpeza de costmaps (planner sem caminho; BLOQUEIO_LOCAL) | autorizada | — |
| **Ré** (inclusive os 4 cm do CONTORNAR) | **BLOQUEADA**: cobertura traseira real e ICP não validados no robô | `ros2 param set /destravar permitir_re true` |
| Ré seguida de giro sem reavaliar (`re_girar` do v3) | **removida** | — |
| Giro frontal experimental | desligado (como estava) | — |

**Custo honesto do bloqueio da ré:** com uma caixa a menos de ~10 cm do para-choque ou numa quina, a deriva do
deslize não deixa lateral útil e a ré é a única manobra que abre espaço → **falha explícita** (o v3 tentava
deslizar e contava com o CM). Na simulação: caixa a 8 cm e quina a 2,5 cm terminam em falha sem ré e são
resolvidas com ré liberada (ré verificada → reavaliação → retomar).

## 7. Testes (offline) — `bash ferramentas/testes/validar_nav_integrada.sh`

**237 aprovados, 0 falhas, 0 pulados** (`resultados/pytest.xml`). Inclui os 165 que já existiam (dois
testes de `test_lateral_ws.py` ajustados à árvore nova, mantendo a intenção: o validado é o restante do
caminho seguido; o patch PRIORIDADE_LATERAL continua reprodutível sobre a árvore anterior preservada).

| Pedido | Teste |
|---|---|
| 1 caminho livre | `test_caminho_livre_chega_sem_replanejar[3 árvores]`, `test_caminho_livre_e_curva[open]` |
| 2 desvio antecipado | `test_replaneja_com_obstaculo_novo_sem_parar_o_seguimento`, `test_integrada_confere_o_caminho_a_2hz`, `test_scan_novo_altera_o_planejamento_global` |
| 3 caixa parcial | `test_caixa_parcial_com_contorno_normal_devolve_ao_nav2_sem_mexer`, `test_suavizacao_em_colisao_usa_o_caminho_original_seguro` |
| 4 obstáculo novo | `test_obstaculo_no_trecho_ja_percorrido_nao_replaneja`, `test_caminho_valido_preservado_ate_15_s`, `test_sem_intersecao_com_caixa_no_laco_ideal[dynamic]` |
| 5 lateral | `test_oscilacao_vira_escape_lateral_verificado_pelo_icp`, `test_lateral_escolhe_lado_pela_continuacao_nao_pelo_raio_mais_longo`, `test_mecanum_lateral_sem_girar`, `test_dwb_real_..._lateral_e_diagonal` |
| 6 quina | `test_quina_detectada_pelo_canto`, `test_quina_sem_re_falha_explicitamente_com_causa`, `[corner]` |
| 7 T | `test_cruzamento_em_t_desliza_para_o_ramo_sem_girar` |
| 8 sem trajetória | `test_falha_do_controle_chama_destravar_e_replaneja[105,106]`, `test_planner_sem_caminho_limpa_global_...` |
| 9 CM bloqueando | `test_collision_monitor_bloqueando_movimento`, `test_collision_monitor_inativo_...`, ROS `test_cm_inativo_aborta_sem_mover` |
| 10 ré recusada | `test_re_recusada_por_padrao_...`, `_sem_cobertura_traseira`, `_com_obstaculo_atras`, `_quando_o_icp_nao_conseguiria_confirmar` |
| 11 ré + reavaliação | `test_re_seguida_de_reavaliacao_em_simulacao`, `test_re_nao_confirmada_nao_vira_sucesso` |
| 12 giro recusado | `test_giro_recusado_por_falta_de_espaco`, `test_giro_que_nao_resolveu_nao_se_repete`, `test_lateral_preferido_ao_giro_quando_resolve` |
| 13 sem progresso | `test_recuperacao_sem_progresso_detecta_retorno_ao_mesmo_bloqueio`, `test_retomar_nao_se_repete_no_mesmo_lugar` |
| 14 cancelamento | `test_cancelamento_durante_recuperacao_...`, `_durante_giro`, `test_goal_novo_interrompe_a_recuperacao`, ROS `test_cancelamento_durante_manobra_para` (cancelado no meio do lateral, último comando zero) |
| 15 falha segura | `test_sem_alternativa_falha_explicita_sem_mover`, `..._na_primeira_recusa`, `test_tentativas_limitadas_...` |
| 16 orientação final | `test_orientacao_final_obrigatoria` (DWB real), `test_checkers_footprint_e_collision_monitor_preservados` |

Níveis de prova: **(a)** simulação 2D do `Recuperador` (raycast 5,9 Hz, 30–35% de raios `inf`, odom que
superestima o lateral, deriva, polígonos do CM, colisão do footprint; zero colisões em todos os casos e
sementes); **(b)** interpretador da semântica BT.CPP/Nav2 com servidores simulados, para as três árvores;
**(c)** plugins reais (DWB, Navfn, costmaps, BT.CPP) em memória; **(d)** `destravar` v4 **real** como
processo ROS num domínio isolado (166–171, localhost; o robô está no 0). Os testes ROS acharam dois defeitos
que a simulação não pegava (atributo `handle` reservado do `rclpy` e severidade de log), ambos corrigidos.

### Comparativo (`resultados/comparativo.md`)

| Caso (árvores, servidores simulados) | Integrada | Anterior | Reduzida |
|---|---|---|---|
| obstáculo novo no restante: atraso do replanejamento | 0,2 s | 0,7 s | 0,2 s |
| obstáculo só no trecho já percorrido: planejamentos | 1 | 6 (à toa) | 1 |
| suavização colide, original válido | segue o original | segue o suavizado **sem checar** | **falha** |
| DWB sem trajetória 1× | recupera | recupera | **falha** |
| recuperação impossível | falha após 1 destravar | 4 destravar + 2 giros + 4 limpezas | falha sem tentar |

| Controle (laço ideal, sem CM/física) | Shim | DWB direto |
|---|---|---|
| lateral 10 cm | 12,5 s, gira 1,48 rad | 0,7 s, sem giro |
| curva | 9,7 s | 7,1 s |
| caixa (obstacle/dynamic/corner), 30 s | não conclui | **não conclui** |
| near (caixa a 2,5 cm) | 1 interseção | 0 |

**Limite medido:** DWB direto sozinho, com os críticos atuais, **não conclui o contorno da caixa** no laço
ideal. A candidata mostrou que GoalAlign 18→0 (`early_test`, perfil que está rodando agora) concluiu a caixa
existente. É o próximo passo separado, como pedido. Desvio antecipado completo depende dele e da validação no
robô.

## 8. Verificar a configuração ativa (só leitura)

```bash
source /opt/ros/jazzy/setup.bash && source ~/robo_jazzy_ws/install/setup.bash
ros2 param get /controller_server FollowPath.plugin            # dwb_core::DWBLocalPlanner
ros2 param get /controller_server FollowPath.GoalAlign.scale   # 18.0 (integrada) | 0.0 (early_test)
ros2 param get /bt_navigator default_nav_to_pose_bt_xml        # .../navigate_to_pose_smooth.xml
ros2 param get /global_costmap/global_costmap obstacle_layer.scan.obstacle_max_range   # 3.5
ros2 param get /destravar permitir_re                          # False (so existe no v4)
ros2 topic info -v /cmd_vel_nav | grep "Node name"
python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_nav_integrada.py --conferir
# e no terminal/log do Nav2 (~/logs/<hora>/) deve aparecer: "Destravar v4 pronto ... re BLOQUEADA"
```

## 9. Ativação controlada (quando não houver missão)

1. Parar a sessão de teste atual (Ctrl+C no terminal do `navigation_test.launch.py`). Até lá ela segue com
   `early_test`, sem recuperação, e com o `destravar` v3 em memória.
2. `~/subir_robo.sh <mapa>` (usa `navigation.launch.py` → YAML e árvore integrados, destravar v4).
3. Conferir com os comandos do item 8 **antes** do primeiro goal.
4. Robô parado diante de uma caixa: `python3 ~/robo_jazzy_ws/ferramentas/testes/conferir_manobras_parado.py`
   (só leitura) mostra o que seria autorizado e por quê.
5. Um teste por bag (`~/gravar_teste.sh`; grava `--all`, inclui `/destravar/estado` e `/rosout`):
   (a) caminho livre; (b) caixa parcial; (c) caixa bloqueando; (d) T. Medir: tempo, inversões, se houve
   `DECISAO`, `RESULTADO ... ok` com ICP, colisão/encosto.
6. **Liberar a ré só depois** de, com o robô parado em 3 lugares reais (perto de parede, de caixa, no
   corredor), o item 4 mostrar a traseira ≥ 90% observada e o ICP ok, e de um teste com uma caixa a 8 cm
   gravado: `ros2 param set /destravar permitir_re true` (vale até reiniciar).

**Medir antes de confiar no lateral perto de obstáculo:** `deriva_lateral` 0,4 vem de 06/11 (k 0,31–0,37 a 0,08 m/s), mas na missao32 o deslize virou avanço com k 0,8–1,3 (Encosto). Se for assim na velocidade do destravar, o volume varrido do lateral está otimista. Durante o deslize continuam valendo a parada pelo LiDAR (< 4 cm na direção com deriva) e o polígono do CM (5 cm). O log `RESULTADO lateral` traz o dx medido pelo ICP: use o primeiro bag para ajustar `ros2 param set /destravar deriva_lateral <k>`.

Ajustes em tempo de execução, sem reiniciar: `ros2 param set /destravar permitir_lateral|permitir_giro
false`, `deriva_lateral 0.35`, `cobertura_lateral 0.6`.

## 10. Reversão

```bash
# com a navegação parada
cd ~/robo_jazzy_ws && python3 ferramentas/patches/aplicar_nav_integrada.py --desfazer
colcon build --packages-select robo_navigation && source install/setup.bash
```

Volta os 6 arquivos ao backup mais recente (testado numa cópia: hashes idênticos ao original). Para voltar
só o destravar: `cp src/robo_navigation/scripts/destravar_node.py.bak_navintegrada_20261110_105242
src/robo_navigation/scripts/destravar_node.py` (o v3 não usa `recuperacao_avancada.py`). Para usar a árvore
antiga com o YAML novo: lançar com `default_nav_to_pose_bt_xml` apontando para
`navigate_to_pose_smooth_anterior_20261110.xml`. Backup completo: `backups/integracao_nav_20261110_102801/`.

## Arquivos

Alterados (com `.bak_navintegrada_20261110_105242`): `config/nav2_params.yaml`,
`behavior_trees/navigate_to_pose_smooth.xml`, `scripts/destravar_node.py`, `test/test_navigation_profiles.py`,
`test/test_lateral_ws.py`, `CMakeLists.txt`, `package.xml`.
Novos: `scripts/recuperacao_avancada.py`, `behavior_trees/navigate_to_pose_smooth_anterior_20261110.xml`,
`test/{sim_recuperacao,bt_mini,ros_cenario_destravar}.py`,
`test/test_{recuperacao_avancada,arvore_integrada,config_integrada,destravar_ros_isolado}.py`,
`test/sondas_nav2/` (fontes das sondas da candidata + `setblackboard`, `COLCON_IGNORE`),
`ferramentas/patches/aplicar_nav_integrada.py`,
`ferramentas/testes/{validar_nav_integrada.sh,comparar_nav_integrada.py,conferir_manobras_parado.py}`.
Não tocados: alinhador, filtro, suavizador, hardware, firmware, Collision Monitor, perfis/árvore/launch de
teste, `candidatos/`.
