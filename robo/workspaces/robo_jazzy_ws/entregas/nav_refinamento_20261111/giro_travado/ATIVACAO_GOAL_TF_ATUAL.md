# GOAL_TF_ATUAL: ativação supervisionada e rollback

**Estado em 11/11:**
- **aplicado no `src/`** (árvore principal e `nav2_params.yaml`);
- **compilado e instalado** (`robo_bt_plugins`, `robo_navigation`);
- **NÃO ativo no robô.** O Nav2 em execução desde 09:33 carregou a árvore anterior e não a recarrega sozinho: `always_reload_bt_xml` está no padrão `false`. Ele também não tem a biblioteca nova carregada.

**Atenção.** Até o reinício, um ciclo de lifecycle do Nav2 *sem* reiniciar os processos (deactivate/activate, ou reset pelo lifecycle_manager) faria o `bt_navigator` reler a árvore nova sem a biblioteca nova, e a navegação pararia. Para ativar, reinicie o `~/subir_robo.sh` inteiro, como abaixo.

## O que mudou (uma coisa só)

O `FollowPath` passa a receber `{caminho_controlador}`, uma **cópia** do `{smoothed_path}` feita pelo nó `CaminhoSemCarimbo`. A cópia tem o carimbo de tempo zerado no header e em cada pose; frame, posições e orientações ficam iguais.

O `{smoothed_path}` (usado por `TruncatePathLocal`/`IsPathValid`), o `{path}` do planner e a rota do mapa do alinhador **não** mudam.

Efeito: o goal checker do `controller_server` converte o fim do caminho com o `map→odom` **mais recente**, igual ao DWB.

Quando ele ajuda: quando o goal é alcançado **menos de 10 s depois do último (re)planejamento**, com o SLAM corrigindo o `map→odom` nesse intervalo. Acima de 10 s, o próprio Nav2 já cai no TF mais recente (fallback do `nav_2d_utils::transformPose`).

**Limitações:**
- Carimbo zero **não corrige saltos de localização**: o alvo segue o `map→odom` atual, com os mesmos saltos do SLAM.
- Com carimbo zero, a conversão usa sempre o último `map→odom`. Isso já é o que o DWB faz com esse mesmo caminho.

## 1. Encerrar a sessão atual (só o operador, com o robô parado)

```bash
# Em outro terminal, confirme que nao ha goal em andamento (status 2 = executando):
source /opt/ros/jazzy/setup.bash
timeout 10 ros2 topic echo --once /navigate_to_pose/_action/status | grep "status:"   # sem saida ou 4/5/6 = nada ativo
# Confirme que nao ha gravacao em curso (o gravar_teste.sh roda em terminal proprio):
pgrep -af "ros2 bag record" || echo "sem gravacao"
```

Depois, no terminal do `~/subir_robo.sh`, use `Ctrl+C` e espere todos os nós saírem: `pgrep -af "nav2_|robo_hardware"` não deve mostrar nada.

## 2. Subir com a versão nova

```bash
cd ~/robo_jazzy_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
~/subir_robo.sh <mapa>
```

## 3. Verificar que a versão carregada é a nova (antes de qualquer goal)

```bash
source /opt/ros/jazzy/setup.bash
ros2 param get /bt_navigator plugin_lib_names
#   tem de listar robo_caminho_original_seguro_bt_node e robo_caminho_sem_carimbo_bt_node
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
grep -c 'CaminhoSemCarimbo' $(ros2 param get /bt_navigator default_nav_to_pose_bt_xml | awk '{print $NF}')   # 1
grep -o 'librobo_caminho_[a-z_]*\.so' /proc/$(pgrep -f nav2_bt_navigator/bt_navigator)/maps | sort -u
#   tem de listar as 2 bibliotecas (a sem_carimbo so aparece se ela foi carregada)
ros2 lifecycle get /bt_navigator   # active
```

**No 1º goal (supervisionado), confirme o nó em uso e o carimbo zero:**
```bash
ros2 topic echo /behavior_tree_log | grep -m1 CaminhoParaOControlador
ros2 topic echo --once /received_global_plan --field header.stamp    # sec: 0  nanosec: 0
```

Se algum item falhar, **não navegue**. Faça o rollback (seção 5).

## 4. Teste físico (um bag por versão, nunca com goal automático)

- **BASELINE:** antes de ativar, com a versão atual, ou com o rollback aplicado.
- **CORRIGIDO:** depois desta ativação.
- Fora isso, tudo igual: `giro_final_comum false`, `rota_mapa_obstaculo ignorar`, Patch 01 em `registrar`, `virar_caminho_graus 45`.

**Montagem:**
- área aberta de pelo menos 2 × 2 m, a mais de 0,8 m de paredes, sem caixas;
- operador com parada de emergência; Collision Monitor ativo; ré bloqueada;
- gravar com `~/gravar_teste.sh <nome> "<descricao>"` em terminal separado.

**Roteiro:**
- 6 goals sem área, enviados à mão pelo RViz, a cerca de 2 m um do outro;
- yaw do goal +90° (anti-horário) em relação à chegada nos goals 1, 3 e 5, e −90° nos goals 2, 4 e 6;
- nenhuma troca de goal antes de terminar; até 60 s por goal.

**Sucesso (todos):**
1. 6/6 com "Reached the goal!" e NavigateToPose SUCCEEDED.
2. Erro de yaw final ≤ 10°.
3. Nenhum giro de mais de 30° além do yaw pedido.
4. Giro final concluído em ≤ |erro inicial| / 0,16 rad/s + 3 s.
5. Sem toque, sem parada de emergência, sem intervenção.

**Aviso:** o defeito só aparece se o SLAM corrigir o `map→odom` nos ~10 s entre o último replanejamento e a chegada. Em área aberta isso pode não acontecer, e aí a baseline também passa e o teste **não discrimina**. A prova do mecanismo é o replay (`bash ~/robo_jazzy_ws/ferramentas/testes/validar_giro_travado.sh`). Ele é obrigatório e passa 6/6 com o código real do Nav2:
- 12:37: a versão atual nunca encerra; a corrigida encerra às 12:37:29,30;
- bag das 11:48: 5/5 "Reached" reais reproduzidos nas duas versões.

**Abortar:** robô a < 10 cm de algo, giro > 360° ou comportamento inesperado → parada de emergência e anote a hora.

## 5. Rollback (testado numa cópia: volta byte a byte ao estado anterior)

```bash
python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_goal_tf_atual.py --desfazer
cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash
# reiniciar o ~/subir_robo.sh (passos 1 e 2)
```

A biblioteca `robo_caminho_sem_carimbo_bt_node` pode ficar instalada: sem o `plugin_lib_names`, ela não é carregada.

A cópia de antes está em `entregas/nav_refinamento_20261111/backups/goal_tf_atual_<data>/`, com `SHA256_pre.txt`, `SHA256_pos.txt` e o `git status`.

## Não mudou (conferido)

- `decel_lim_theta` −0,2; `min_speed_theta` 0,15; `max_vel_theta` 0,16; `GoalAlign.scale` 18;
- goal checker (0,06 m / 0,17 rad, stateful); Collision Monitor;
- destravar v4 e recuperação avançada (ré bloqueada, `permitir_re` false); suavizador; launch;
- alinhador (`giro_final_comum` false, `rota_mapa_obstaculo` ignorar, `virar_caminho_graus` 45, `fundido_k_ff` 0,2); perfil GoalAlign 0 não usado.

A trava de giro anti-horário do DWB continua. Ela só vira giro sem fim se o goal checker não encerrar, que é o que esta correção trata.
