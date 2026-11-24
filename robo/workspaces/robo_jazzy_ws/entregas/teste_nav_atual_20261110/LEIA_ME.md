# Teste integrado ao src atual — 10/11/2026

Agora os arquivos de teste estão em `~/robo_jazzy_ws/src/robo_navigation`. O lançamento padrão e os YAML/XML padrões foram preservados byte a byte; não houve reinício de nós nem movimento. Esta entrega adiciona uma entrada explícita para o próximo teste no robô, sem carregar o overlay experimental.

## Iniciar o teste

Com a instância anterior de navegação encerrada pelo operador, no próximo início da navegação:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
ros2 launch ~/robo_jazzy_ws/src/robo_navigation/launch/navigation_test.launch.py
```

Não executar uma segunda navegação em paralelo. Mantenha os produtores usuais de hardware, scan, mapa e TF do robô. O novo launch reproduz a cadeia do lançamento local atual, incluindo filtro, suavizadores, Collision Monitor e alinhador, sem editar seus códigos. Ele é chamado por caminho absoluto porque os arquivos novos não foram instalados sobre o install operacional. Não precisa recompilar para esse comando.

O perfil padrão desse lançamento específico é `early_test`:

- YAML `config/nav2_params_early_test.yaml`: DWB direto, global consolidado preservando valores efetivos; GoalAlign=0 e sim_time=1,6 s. Velocidades, acelerações, footprint e Collision Monitor iguais aos atuais.
- BT `behavior_trees/navigate_to_pose_early_test.xml`: consulta do caminho restante a 2 Hz, preservação de caminho válido, novo plano quando necessário, smoothing com checagem de colisões.
- **Sem ações automáticas de recuperação nessa BT**: se não há trajetória viável, retorna falha. Não usa o servidor experimental nem depende de pose independente para a navegação normal.

Para isolar primeiro apenas retirada do Shim, preservando críticos e horizonte:

```bash
ros2 launch ~/robo_jazzy_ws/src/robo_navigation/launch/navigation_test.launch.py nav_profile:=dwb_test
```

Começar com objetivo Nav2 direto (por exemplo, Nav2 Goal no RViz), área livre e depois caixa com passagem ampla. Isso avalia o controlador e planner. A sequência de missão WS mantém os comportamentos próprios do alinhador, inclusive seus giros e eventuais manobras; não confundir ausência de ações de recuperação nesta BT com bloqueio de todos os publicadores externos. O alinhador foi preservado.

## Confirmar em memória após iniciar

```bash
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.sim_time
ros2 param get /controller_server FollowPath.GoalAlign.scale
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /global_costmap/global_costmap plugins
ros2 param get /global_costmap/global_costmap obstacle_layer.scan.topic
```

Esperado no perfil early_test: DWBLocalPlanner, 1,6 s, GoalAlign 0, BT terminando em navigate_to_pose_early_test.xml, static/obstacle/inflation e /scan_nav. Parâmetros no arquivo não provam o que um nó já iniciado carregou. Goals externos ainda podem selecionar outra BT.

## Voltar ao comportamento anterior

No próximo início, usar seu comando habitual com `navigation.launch.py`. Nenhum arquivo padrão precisa ser restaurado. Não extrair backups por cima do src nem substituir o alinhador.

## Verificação realizada

- Testes de resolução real dos argumentos e parâmetros do launch, invariantes DWB e regressões laterais existentes: `tests.log`.
- BT registrada/verificada com plugins Nav2 instalados: `bt.log`.
- LaserScan→ObstacleLayer/InflationLayer→Navfn com o YAML adicionado ao src: `scan_replan.log`; rota reta passou a desviar e não atravessou células proibidas no ensaio.
- Controlador DWB real configurado com o novo YAML: `lateral.json`, chegada lateral em 0,8 s sem yaw no modelo ideal.
- CLI de lançamento inspecionada com `--show-args`, sem iniciar nós: `launch_arguments.log`.
- Backup do estado anterior e verificação dos arquivos preservados: `preservacao.json`.

Os testes são offline, não comprovação física. Recuperação avançada permanece na candidata isolada e não foi ligada nesta primeira etapa de teste controlado. O novo launch foi copiado do lançamento local atual para preservá-lo integralmente; mudanças futuras na composição do lançamento principal devem ser revisadas também nesta entrada de teste.
