# Navegação candidata — Robo-WORK

Candidata criada em `~/robo_jazzy_ws/candidatos/navegacao_20261110_094606`, a partir do `src` local em 10/11/2026. Não foi criada apenas a partir do HEAD. O workspace operacional e seu `install` não foram alterados.

Arquivos principais:

- `src/robo_navigation/config/nav2_params.yaml`: DWB direto + global consolidado + variante antecipativa medida.
- `src/robo_navigation/config/nav2_params_dwb_candidate.yaml`: etapa isolada DWB direto, sem ajuste de críticos/horizonte/velocidades.
- `src/robo_navigation/behavior_trees/navigate_to_pose_candidate.xml`: planejamento condicional e recuperação integrados.
- `src/robo_navigation/launch/navigation_candidate.launch.py`: seleciona explicitamente os dois primeiros arquivos principais no perfil `candidate`.
- `src/robo_recovery/robo_recovery/supervised.py` e `supervised_node.py`: decisão geométrica e execução supervisionada.
- `scripts/install_candidate.sh`, `validate_offline.sh`, `verify_loaded_readonly.sh`: instalação isolada, validação e leitura do estado carregado.

O marcador `COLCON_IGNORE` impede descoberta acidental desta cópia em um build recursivo do workspace pai. Os scripts usam `--base-paths src` dentro da candidata.

## Instalação reversível, já executada somente na candidata

```bash
cd ~/robo_jazzy_ws/candidatos/navegacao_20261110_094606
bash scripts/install_candidate.sh
```

Produz `build_candidate/` e `install_candidate/`, sem copiar nada para `~/robo_jazzy_ws/install`. Não registrar esse overlay no `.bashrc` e não usar o `navigation.launch.py` legado da cópia: ele foi preservado como snapshot, mas não integra a nova BT/recuperação.

Para preparar uma sessão isolada de inspeção, em terminal novo:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/candidatos/navegacao_20261110_094606/install_candidate/setup.bash
ROS_DOMAIN_ID=166 ros2 launch robo_navigation navigation_candidate.launch.py profile:=candidate
```

Esse lançamento mantém lifecycle inativo, exige domínio 166, usa `/mecanum_candidate`, remapeia scan/TF/odom e não inicia hardware, alinhador ou identificação WS. Não foi executado nesta entrega. Para navegação fechada precisa dos produtores de mapa, TF, relógio e scans do simulador isolado. Não apontar seus tópicos para sensores/atuadores operacionais. Esta preparação não é ativação física.

Perfis, para isolar efeitos: `shim` → `global` (somente consolidação) → `dwb` (retirada do wrapper) → `early` (GoalAlign 18→0) → `predictive` (horizonte 0,8→1,6). Esses perfis usam a mesma BT A/B sem recuperação. `integrated-predictive` usa a árvore experimental anterior; `candidate` usa a nova árvore condicional, sem limpeza automática, com recuperação supervisionada. `stable` é um alias de árvore com o mesmo comportamento da candidata nesta revisão.

A navegação normal conserva o progress checker do Nav2. `supervise_navigation_progress=false` evita exigir a certificação própria de recuperação para encaminhar comandos normais. A supervisão independente adicional é opt-in. Essa opção não autoriza recuperação: ré/lateral/giro continuam exigindo scan/TF recentes, cobertura calibrada, pose independente certificada, volume varrido livre e parada comprovada. Não usar odometria derivada de comandos como prova de translação. Giro permanece desabilitado por padrão. CM STOP, conflitos de publicadores, timeout e falhas ficam travados; nova action não limpa uma falha.

## Verificar o que foi carregado — somente leitura

No terminal/ambiente ROS que vê a navegação operacional:

```bash
cd ~/robo_jazzy_ws/candidatos/navegacao_20261110_094606
bash scripts/verify_loaded_readonly.sh
ros2 param dump /controller_server
ros2 param dump /bt_navigator
ros2 param dump /global_costmap/global_costmap
```

Para uma sessão candidata já preparada em outro terminal:

```bash
ROS_DOMAIN_ID=166 bash scripts/verify_loaded_readonly.sh /mecanum_candidate
```

Esperado na candidata: `FollowPath.plugin=dwb_core::DWBLocalPlanner`, `sim_time=1.6`, `GoalAlign.scale=0.0`; BT terminando em `navigate_to_pose_candidate.xml`; plugins globais static/obstacle/inflation; scan remapeado dentro do namespace isolado; marcação ativa e alcance 3,5 m. Na configuração base ainda ativa espera-se RotationShim e `navigate_to_pose_smooth.xml`.

`ros2 pkg prefix robo_navigation` informa o overlay do terminal, não a configuração de um processo já iniciado. Parâmetros em memória e caminho real da BT são evidências mais fortes. Um cliente pode sobrescrever a árvore em cada goal; a árvore efetiva de cada ação exige também inspecionar o goal/log do cliente. Inscrição em scan não prova marcação; o teste integrado de scan/planner fica em `review/scan_replan_result.log`.

## Validação e reversão

```bash
bash scripts/validate_offline.sh
```

O script calcula trajetórias em memória, testa callbacks/BT e não envia goals ou comandos de movimento. A matriz e limites estão em `review/RELATORIO.md` e `review/COMPARATIVO.md`. As bags foram analisadas em leitura, sem replay no domínio do robô.

Para abandonar a instalação, fechar somente os terminais isolados e abrir um terminal com o overlay operacional usual. Não é necessário restaurar qualquer arquivo operacional, pois nenhum foi substituído. Não extrair o backup por cima do workspace: ele preserva mudanças locais, inclusive alinhador, e não autoriza sobrescrever trabalho posterior.

Backup inicial: `~/robo_jazzy_ws/backups/nav_candidata_20261110_094606/`, com status completo, patch binário, HEAD, arquivo tar e hashes verificados de 426 arquivos Git visíveis, incluindo não rastreados. Build/install/cache ignorados pelo Git não fazem parte desse tar. A candidata contém cópia do src local; conteúdo protegido pode ser verificado por `tests/mecanum/test_delivery.py`.

Ativação física futura precisa validar a cadeia completa e sua composição com o alinhador preservado. Não foi criado comando que reinicie os nós ou substitua o lançamento operacional. O launch candidato seleciona a BT explicitamente porque trocar somente YAML no launch legado deixaria `navigate_to_pose_smooth.xml` selecionado.
